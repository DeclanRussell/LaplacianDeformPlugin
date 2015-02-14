#include "LaplacianMeshDeformerNode.h"
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MGlobal.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MVectorArray.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MItGeometry.h>
#include <maya/MDagModifier.h>
#include <maya/MIntArray.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MMatrix.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFnFloatArrayData.h>
#include <maya/MFloatArray.h>
#include <iostream>

// You MUST change this to a unique value!!!  The id is a 32bit value used
// to identify this type of node in the binary file format.  
//
MTypeId     LaplacianMeshDeformer::m_id( 0x090001 );

// The attributes of our node.
MObject		LaplacianMeshDeformer::m_localCoord;
MObject     LaplacianMeshDeformer::m_association;
MObject     LaplacianMeshDeformer::m_handleTransformMatrix;
MObject		LaplacianMeshDeformer::m_numSets;
MObject     LaplacianMeshDeformer::m_handlesAdded;
MObject     LaplacianMeshDeformer::m_recompute;

LaplacianMeshDeformer::LaplacianMeshDeformer() {}
LaplacianMeshDeformer::~LaplacianMeshDeformer() {}

MStatus  	LaplacianMeshDeformer::deform(MDataBlock& 	_data, MItGeometry& 	_iter,   const MMatrix& 	_mat,  unsigned int		_multiIndex)
{
	MStatus stat;
	unsigned int i,j;


    //if we've manualy called compute set bool to false again
    MDataHandle recomputeHandle = _data.inputValue(m_recompute, &stat);
    recomputeHandle.set(false);

    // Envelope data from the base class.
    // The envelope is simply a scale factor.
    MDataHandle envelopeData = _data.inputValue(envelope, &stat);
	if (MS::kSuccess != stat) return stat;
	float envelopeValue = envelopeData.asFloat();	


    // This is our imput mesh, imputMeshObj is our mesh object which we can use to get our verts from.
    // inputGeom = the input geomtry. This static member is inherited from MPxDeformerNode
    MArrayDataHandle inputMeshHandle = _data.inputArrayValue( input, &stat );
    inputMeshHandle.jumpToElement(0);
    // This is like an array of our input selected geometry. It has children which are the mesh handles
    MDataHandle inputMeshElementHandle = inputMeshHandle.inputValue(&stat);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(inputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &stat);
	unsigned int numVertices = fnInputMesh.numVertices();
	MPointArray origMeshVertexArray(numVertices);
	fnInputMesh.getPoints(origMeshVertexArray);



    // Get the association between vertices and handle locators
    MDataHandle associationHandle = _data.inputValue(m_association, &stat);
	MObject	associationObj = associationHandle.data();
    if(associationObj.isNull()){
        MGlobal::displayWarning("Association Not Set");
        return MS::kSuccess;
    }
	MFnIntArrayData fnAssociation(associationObj, &stat);
	MIntArray assoArray = fnAssociation.array(&stat);  // *******************

	// 2. get the local coordinates for each vertices according to their handles
    MDataHandle localCoordHandle = _data.inputValue(m_localCoord, &stat);
	MObject localCoordObj = localCoordHandle.data();
    if(localCoordObj.isNull()){
        MGlobal::displayWarning("Local Coordinates not set");
        return MS::kSuccess;
    }
	MFnPointArrayData fnLocalCoord(localCoordObj, &stat);
	MPointArray localCoordArray = fnLocalCoord.array(&stat);   // *******************



    //----------------------------------------------------------------------
    //-----------------Create our laplace and delta matrix------------------
    //----------------------------------------------------------------------

    // Check to see if our matricies have been created
    MPoint	tmpPoint;
    if(m_laplaceMatrix.nonZeros()==0)
	{
        /// this laplacian Coord Array is what he uses to store his delta values;
        MIntArray	oneRingNeighbours;
        MItMeshVertex itOrigMeshVertex(inputMeshObj);
        unsigned int numNeighbours;
		itOrigMeshVertex.reset();
        /// This is where he creates his Laplace matrix. Also add's the anchors apparently
        m_laplaceMatrix.resize(numVertices, numVertices);
        m_laplaceMatrix.setIdentity();
        m_deltaMatrix.resize(numVertices,3);
        MPoint delta;

        int prevN, nextN;
        MVector e1,e2;
        double alpha,beta,weight,weightSum;

        for(i=0;i<numVertices;i++)
		{
            //Lets add our information to our laplace matrix
            //Get our ring of neighbours around our vertex
            stat = itOrigMeshVertex.getConnectedVertices(oneRingNeighbours);
            numNeighbours = oneRingNeighbours.length();
            //set our weighted sum to 0
            tmpPoint.x = tmpPoint.y = tmpPoint.z = 0.0;
            weightSum = 0.0;
            for(j=0;j<numNeighbours;j++){

                //calculate out cotangent weights
                //cotangent weights better preserve the shape of our mesh when deformed
                //get our previous and next neightbours
                (j==0) ? prevN = numNeighbours-1 : prevN = j-1;
                (j==numNeighbours-1) ? nextN = 0 : nextN = j+1;

                //calculate our alpha angle
                e1 = origMeshVertexArray[oneRingNeighbours[j]] - origMeshVertexArray[oneRingNeighbours[nextN]];
                e2 = origMeshVertexArray[i] - origMeshVertexArray[oneRingNeighbours[nextN]];
                alpha = acos((e1*e2)/(e1.length() * e2.length()));

                //calculate our beta angle
                e1 = origMeshVertexArray[oneRingNeighbours[j]] - origMeshVertexArray[oneRingNeighbours[prevN]];
                e2 = origMeshVertexArray[i] - origMeshVertexArray[oneRingNeighbours[prevN]];
                beta = acos((e1*e2)/(e1.length() * e2.length()));

                //calculate our final weight of the neighbour vertex
                weight =((1.0/tan(alpha)) + (1.0/tan(beta))) / 2.0;
                m_laplaceMatrix.coeffRef(i,oneRingNeighbours[j]) = -weight;
                tmpPoint += origMeshVertexArray[oneRingNeighbours[j]] * weight;
                weightSum+=weight;
            }
            for(j=0;j<numNeighbours;j++){
                m_laplaceMatrix.coeffRef(i,oneRingNeighbours[j])/=weightSum;
            }
            // calculate our final delta and add it to our delta matrix
            tmpPoint = tmpPoint/weightSum;
            delta = origMeshVertexArray[i] - tmpPoint;

            m_deltaMatrix.coeffRef(i,0) = delta.x;
            m_deltaMatrix.coeffRef(i,1) = delta.y;
            m_deltaMatrix.coeffRef(i,2) = delta.z;

            //increment to our next vertex in our mesh
			itOrigMeshVertex.next();
		}
        //Create our transpose laplace so that we dont have to do it when we solve.
        //Dont want to waste any computation time in doing it every update
        m_laplaceTransMatrix = m_laplaceMatrix.transpose();

        MGlobal::displayInfo("Laplace and delta matrix created");
    }


    //----------------------------------------------------------------------
    //----------------------Update Handles if changed-----------------------
    //----------------------------------------------------------------------

    // Get the new tranformation matrix of handles
    MArrayDataHandle handleMatrixHandle = _data.outputArrayValue(m_handleTransformMatrix, &stat);
    MDataHandle numSetsHandle = _data.inputValue(m_numSets, &stat);
    unsigned int numSets = numSetsHandle.asInt();

    MMatrix *newMatrix = new MMatrix[numSets];
    handleMatrixHandle = _data.inputArrayValue(m_handleTransformMatrix, &stat);
    for(i=0;i<numSets;i++)
    {
        stat = handleMatrixHandle.jumpToElement(i);
        MDataHandle eachMatrixHandle = handleMatrixHandle.inputValue(&stat);
        newMatrix[i] = eachMatrixHandle.asMatrix();   // *******************
    }

    //if our handles have changed then lets update them
    MDataHandle handlesAddedHandle = _data.inputValue(m_handlesAdded, &stat);
    bool handlesAdded = handlesAddedHandle.asBool();
    /// this is where he adds his anchors to his laplace matrix
    if(handlesAdded){
        MGlobal::displayInfo("Adding Handles");
        // allocate the memory for handlMatrix array attribute
        unsigned int numHandles = handleMatrixHandle.elementCount(&stat);
        if(numHandles==0)
        {
            MMatrix matrixTmp;
            matrixTmp.setToIdentity();
            MArrayDataBuilder handleMatrixArrayBuilder(&_data, m_handleTransformMatrix, numSets, &stat);
            handleMatrixHandle.set(handleMatrixArrayBuilder);
            for(i=0;i<numSets;i++)
            {
                MDataHandle eachHandleMatrixHandle = handleMatrixArrayBuilder.addElement(i, &stat);
                eachHandleMatrixHandle.setMMatrix(matrixTmp);
            }
        }

        // Find our how many vertex handles we need to add
        unsigned int numFixPoints = 0;
        for(i=0;i<numVertices;i++)
        {
            if(assoArray[i]!=-1) // free vertex
            {
                numFixPoints++;
            }
        }
        //remove any previous handles we had
        m_laplaceMatrix.conservativeResize(numVertices,numVertices);
        m_deltaMatrix.conservativeResize(numVertices,3);
        //now make room for our new vertex handles
        m_laplaceMatrix.conservativeResize(numVertices+numFixPoints,numVertices);
        m_deltaMatrix.conservativeResize(numVertices+numFixPoints,3);

        //set our new handle in our laplace matrix
        unsigned int currentRowNum = numVertices;
        for(i=0;i<numVertices;i++)
        {
            if(assoArray[i]!=-1) // free vertex
            {
                m_laplaceMatrix.coeffRef(currentRowNum,i) = 1.0;
                currentRowNum++;
            }
        }

        //Create our transpose laplace so that we dont have to do it when we solve.
        //Dont want to waste any computation time in doing it every update
        m_laplaceTransMatrix = m_laplaceMatrix.transpose();

        // set our new delta values based on the transorm of our handle
        currentRowNum = numVertices;
        for(i=0;i<numVertices;i++)
        {
            if(assoArray[i]!=-1) // free vertex
            {
                tmpPoint = localCoordArray[i]*newMatrix[assoArray[i]];
                m_deltaMatrix.coeffRef(currentRowNum,0) = tmpPoint.x;
                m_deltaMatrix.coeffRef(currentRowNum,1) = tmpPoint.y;
                m_deltaMatrix.coeffRef(currentRowNum,2) = tmpPoint.z;
                currentRowNum++;
            }
        }
        handlesAddedHandle.set(false);
        MGlobal::displayInfo("Handles Added");
    }

    //----------------------------------------------------------------------
    //---------------------Transform our handles----------------------------
    //----------------------------------------------------------------------
    if(numSets>0){
        //update our delta matrix based on the moving handles
        unsigned int currentRowNum = numVertices;
        for(i=0;i<numVertices;i++)
        {
            if(assoArray[i]!=-1) // free vertex
            {
                tmpPoint = localCoordArray[i]*newMatrix[assoArray[i]];
                m_deltaMatrix.coeffRef(currentRowNum,0) = tmpPoint.x;
                m_deltaMatrix.coeffRef(currentRowNum,1) = tmpPoint.y;
                m_deltaMatrix.coeffRef(currentRowNum,2) = tmpPoint.z;
                currentRowNum++;
            }
        }
    }

    //----------------------------------------------------------------------
    //-----------------------Compute our deformation------------------------
    //----------------------------------------------------------------------

    //if we haven't added any handles then we dont need (and cant) solve for new points
    if(numSets==0) return MS::kSuccess;

    // Compute the deformation
    Eigen::SparseMatrix<double> AtA = m_laplaceTransMatrix * m_laplaceMatrix;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Atb = m_laplaceTransMatrix * m_deltaMatrix;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::Upper > solver(AtA);
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> deformedPoints = solver.solve(Atb);
    if(solver.info()!=Eigen::Success){
        stat.perror("ERROR: Something when wrong with the solver. Soz about that :( <3");
        return MS::kFailure;
    }

    //----------------------------------------------------------------------
    //---------------------Set our new postions-----------------------------
    //----------------------------------------------------------------------

    // set the new points of our mesh
    MPointArray newPos(numVertices);
    for(i=0;i<numVertices;i++)
    {
        newPos[i].x = lerp(origMeshVertexArray[i].x,deformedPoints.coeff(i,0),envelopeValue);
        newPos[i].y = lerp(origMeshVertexArray[i].y,deformedPoints.coeff(i,1),envelopeValue);
        newPos[i].z = lerp(origMeshVertexArray[i].z,deformedPoints.coeff(i,2),envelopeValue);
    }
    stat = _iter.setAllPositions(newPos);

    delete [] newMatrix;
	return MS::kSuccess;
}
//----------------------------------------------------------------------------------------------------------------------
float LaplacianMeshDeformer::lerp(float _x, float _y, float _a){
    return (_x*(1.0-_a) + _y*_a);
}
//----------------------------------------------------------------------------------------------------------------------
void* LaplacianMeshDeformer::creator()
{
	return new LaplacianMeshDeformer();
}


MObject& LaplacianMeshDeformer::accessoryAttribute() const

{
    return LaplacianMeshDeformer::m_handleTransformMatrix;
}

MStatus LaplacianMeshDeformer::accessoryNodeSetup(MDagModifier& _cmd)
{
	MStatus stat= MS::kSuccess; 
	return stat;
}

MStatus LaplacianMeshDeformer::initialize()

{
	// This sample creates a single input float attribute and a single
	// output float attribute.
	//
	MFnTypedAttribute	localCoordAttr;
	MFnMatrixAttribute	handleMatrixAttr;
	MFnTypedAttribute	associateAttr;
	MFnNumericAttribute	numSetsAttr;
    MFnNumericAttribute handlesAddedAttr;
	MStatus				stat;

    /// the full name and brief name are strings that canreturn m_handleTransformMatrix; be used in maya scripting
    /// For example you can edit them with deformerName.association
    m_localCoord = localCoordAttr.create( "localCoordinate", "locc", MFnData::kPointArray, MObject::kNullObj, &stat);
	localCoordAttr.setHidden(true);
    m_association = associateAttr.create("association", "Ass", MFnData::kIntArray, MObject::kNullObj, &stat);
	associateAttr.setHidden(true);
    m_handleTransformMatrix = handleMatrixAttr.create("handleMatrix", "hm", MFnMatrixAttribute::kDouble, &stat );
	stat= handleMatrixAttr.setArray(true);
	stat = handleMatrixAttr.setUsesArrayDataBuilder(true);
	stat = handleMatrixAttr.setHidden(true);
    m_numSets = numSetsAttr.create("numSets", "ns",MFnNumericData::kInt, 0, &stat);
    m_handlesAdded = handlesAddedAttr.create("handlesAdded","ha",MFnNumericData::kBoolean,false,&stat);
    m_recompute = handlesAddedAttr.create("recompute","rc",MFnNumericData::kBoolean,false,&stat);

	// Add the attributes we have created to the node
    stat = addAttribute( m_localCoord );
	if (!stat) { stat.perror("addAttribute"); return stat;}
    stat = addAttribute( m_association );
	if (!stat) { stat.perror("addAttribute"); return stat;}
    stat = addAttribute( m_handleTransformMatrix );
	if (!stat) { stat.perror("addAttribute"); return stat;}
    stat = addAttribute( m_numSets );
	if (!stat) { stat.perror("addAttribute"); return stat;}
    stat = addAttribute( m_handlesAdded );
    if (!stat) { stat.perror("addAttribute"); return stat;}
    stat = addAttribute( m_recompute );
    if (!stat) { stat.perror("addAttribute"); return stat;}

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
    // Firstly we want any changes to our handle transforms to change our mesh to deform it
    stat = attributeAffects( m_handleTransformMatrix, outputGeom );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
    // Secondly lets have a bool if we want to manually call deform
    stat = attributeAffects( m_recompute, outputGeom );
    if (!stat) { stat.perror("attributeAffects"); return stat;}

    MGlobal::displayInfo("Laplacian Deformer Node created");
	return MS::kSuccess;
}

