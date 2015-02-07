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

// Example attributes
// 
MObject		LaplacianMeshDeformer::m_localCoord;
MObject     LaplacianMeshDeformer::m_association;
MObject     LaplacianMeshDeformer::m_handleTransformMatrix;
MObject		LaplacianMeshDeformer::m_numSets;
MObject     LaplacianMeshDeformer::m_handlesAdded;
// Declare our static members
Eigen::SparseMatrix<double> LaplacianMeshDeformer::m_laplaceMatrix;
Eigen::SparseMatrix<double> LaplacianMeshDeformer::m_deltaMatrix;
bool LaplacianMeshDeformer::m_laplaceMatrixInit;

LaplacianMeshDeformer::LaplacianMeshDeformer() {}
LaplacianMeshDeformer::~LaplacianMeshDeformer() {}

MStatus  	LaplacianMeshDeformer::deform(MDataBlock& 	_data, MItGeometry& 	_iter,   const MMatrix& 	_mat,  unsigned int		_multiIndex)
{
	MStatus stat;
	unsigned int i,j;

    std::cout<<"deform()"<<std::endl;
	// Envelope data from the base class.
	// The envelope is simply a scale factor.
    MDataHandle envelopeData = _data.inputValue(envelope, &stat);
	if (MS::kSuccess != stat) return stat;
	float envelopeValue = envelopeData.asFloat();	

	// get the undeformed mesh, this is used for envelope only, if the envelop is 1.0, then this is not necessary, the output geometry will be only affected by the cage and its weight
    /// not sure if this is actually used at all if this is for the envelop as the envelop deosn't actually do anything. maybe he forgot to impliment it?
    /// input = Some kind of input attribute inherited from MPxDeformationNode
    MArrayDataHandle inputMeshHandle = _data.inputArrayValue( input, &stat );
	inputMeshHandle.jumpToElement(0);
    /// @brief this is like an array of our input selected geometry. It has children which are the mesh handles
	MDataHandle inputMeshElementHandle = inputMeshHandle.inputValue(&stat);

    /// ok this is our imput mesh, imputMeshObj is our mesh object which we can use to get our verts from.
    /// inputGeom = the input geomtry. This static member is inherited from MPxDeformerNode
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(inputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &stat);
	unsigned int numVertices = fnInputMesh.numVertices();
	MPointArray origMeshVertexArray(numVertices);
	fnInputMesh.getPoints(origMeshVertexArray);

	// allocate the memory for handlMatrix array attribute
    /// our handles is an array/matrix of locators. Verticies have properties that tell you which locator they associate with
    MDataHandle numSetsHandle = _data.inputValue(m_numSets, &stat);
	unsigned int numSets = numSetsHandle.asInt();
    MArrayDataHandle handleMatrixHandle = _data.outputArrayValue(m_handleTransformMatrix, &stat);
	unsigned int numHandles = handleMatrixHandle.elementCount(&stat);
//	if(numSets == 0)
//		return MS::kSuccess;
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

    // compute the deformation
    if(numSets != numHandles){
        std::cout<<"numSet!=NumHandles"<<std::endl;
		return MS::kSuccess;
    }
	// 1. get the association between vertices and handle locators
    MDataHandle associationHandle = _data.inputValue(m_association, &stat);
	MObject	associationObj = associationHandle.data();
    if(associationObj.isNull()){
        std::cout<<"associationObj.isNull()"<<std::endl;
        return MS::kSuccess;
    }
    else{
        std::cout<<"associations set"<<std::endl;
    }
	MFnIntArrayData fnAssociation(associationObj, &stat);
	MIntArray assoArray = fnAssociation.array(&stat);  // *******************

	// 2. get the local coordinates for each vertices according to their handles
    MDataHandle localCoordHandle = _data.inputValue(m_localCoord, &stat);
	MObject localCoordObj = localCoordHandle.data();
    if(localCoordObj.isNull()){
        std::cout<<"localCoordObj.isNull()"<<std::endl;
        return MS::kSuccess;
    }
    else{
        std::cout<<"Local coordinates set"<<std::endl;
    }
	MFnPointArrayData fnLocalCoord(localCoordObj, &stat);
	MPointArray localCoordArray = fnLocalCoord.array(&stat);   // *******************



    //----------------------------------------------------------------------
    //-----------------Create our laplace and delta matrix------------------
    //----------------------------------------------------------------------

    std::cout<<"here"<<std::endl;
	// 5. compute the laplacian coordinates
    /// this laplacian Coord Array is what he uses to store his delta values
	MPointArray laplacianCoordArray(numVertices);
	MIntArray	oneRingNeighbours;
	MItMeshVertex itOrigMeshVertex(inputMeshObj);
	MPoint	tmpPoint;
	unsigned int numNeighbours;
    /// Here he clarifies that you only want to create the laplace matrix if it doesnt already exsist
    if(!m_laplaceMatrixInit)
	{
        std::cout<<"Creating Matrixcies"<<std::endl;
		itOrigMeshVertex.reset();
        /// This is where he creates his Laplace matrix. Also add's the anchors apparently
        m_laplaceMatrix.resize(numVertices, numVertices);
        m_deltaMatrix.resize(numVertices,3);

        int prevNeighbour, nextNeighbour;
        MVector e1,e2;
        MPoint currentN;
        MPoint weightedSum(0.0,0.0,0.0);
        MPoint delta;
        float alpha, beta, weight;
		for(i=0;i<numVertices;i++)
		{
            //Lets add our information to our laplace matrix
            //Get our ring of neighbours around our vertex
            stat = itOrigMeshVertex.getConnectedVertices(oneRingNeighbours);
            numNeighbours = oneRingNeighbours.length();
            //The diagonal of our matrix will always be 1
            m_laplaceMatrix.coeffRef(i,i)=1.0;
            //set our weighted sum to 0
            tmpPoint.x = tmpPoint.y = tmpPoint.z = 0.0;
            weightedSum.x = weightedSum.y = weightedSum.z = 0.0;
            for(j=0;j<numNeighbours;j++){
//                //set our current neighbour
//                currentN = origMeshVertexArray[oneRingNeighbours[j]];

//                //get our previous neightbour and next neighbour indecies
//                (j==0) ? prevNeighbour=numNeighbours-1 : prevNeighbour = j-1;
//                (j==numNeighbours-1) ? nextNeighbour = 0 : nextNeighbour = j+1;

//                //calculate our alpha for our cotangent weights
//                e1 = currentN - origMeshVertexArray[oneRingNeighbours[nextNeighbour]];
//                e2 = itOrigMeshVertex.position() - origMeshVertexArray[oneRingNeighbours[nextNeighbour]];
//                alpha = acos((e1*e2)/(e1.length() * e2.length()));

//                //calculate our beta for our cotangent weights
//                e1 = currentN - origMeshVertexArray[oneRingNeighbours[prevNeighbour]];
//                e2 = itOrigMeshVertex.position() - origMeshVertexArray[oneRingNeighbours[prevNeighbour]];
//                beta = acos((e1*e2)/(e1.length() * e2.length()));

//                //calculate our weight for this vertex
//                weight = 0.5 * (1.0/tan(alpha) + 1.0/tan(beta));
//                weight/=numNeighbours;

//                // bit of a work around for now not sure what is going wrong with these weights
//                if(weight>1) weight=1.0;
//                if(weight<0) weight=0.0;

//                //Add to the weighted sum to create our delta value
//                currentN.x *= weight;
//                currentN.y *= weight;
//                currentN.z *= weight;
//                weightedSum+= currentN;

//                //now we're all done lets add the weight to our matrix
//                m_laplaceMatrix.coeffRef(i,oneRingNeighbours[j]) = weight*-1.0;

//                // lets try with his weighting
                tmpPoint += origMeshVertexArray[oneRingNeighbours[j]];
                m_laplaceMatrix.coeffRef(i,oneRingNeighbours[j]) = -1.0/numNeighbours;
            }

            // calculate our final delta and add it to our delta matrix
//            delta =origMeshVertexArray[i] - weightedSum;
//            m_deltaMatrix.coeffRef(i,0) = delta.x;
//            m_deltaMatrix.coeffRef(i,1) = delta.y;
//            m_deltaMatrix.coeffRef(i,2) = delta.z;
            // lets try with his weighting
            tmpPoint = tmpPoint/numNeighbours;
            delta = origMeshVertexArray[i] - tmpPoint;
            m_deltaMatrix.coeffRef(i,0) = delta.x;
            m_deltaMatrix.coeffRef(i,1) = delta.y;
            m_deltaMatrix.coeffRef(i,2) = delta.z;

            std::cout<<"My delta "<<delta.x<<","<<delta.y<<","<<delta.z<<std::endl;

			laplacianCoordArray[i] = origMeshVertexArray[i] - tmpPoint;
            std::cout<<"His delta "<<laplacianCoordArray[i].x<<","<<laplacianCoordArray[i].y<<","<<laplacianCoordArray[i].z<<std::endl;

            //increment to our next vertex in our mesh
			itOrigMeshVertex.next();
		}
        // declare that we have made our laplace matrix so we dont recalcuate it
        std::cout<<"Congratulations! Laplacian Matrix has been created"<<std::endl;
        std::cout<<"Laplace matrix is of size "<<m_laplaceMatrix.rows()<<","<<m_laplaceMatrix.cols()<<std::endl;
        std::cout<<"Delta matrix is of size "<<m_deltaMatrix.rows()<<","<<m_deltaMatrix.cols()<<std::endl;

        //std::cout<<"It looks something like this:"<<std::endl;
        //std::cout<<m_laplaceMatrix<<std::endl;
        m_laplaceMatrixInit = true;
	}

    std::cout<<"here 1"<<std::endl;

    //----------------------------------------------------------------------
    //----------------------Update Handles if changed-----------------------
    //----------------------------------------------------------------------

    // Get the new tranformation matrix of handles
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
    std::cout<<"handles Added: "<<handlesAddedHandle.asBool()<<std::endl;
    /// this is where he adds his anchors to his laplace matrix
    if(handlesAdded){
        std::cout<<"why am I here?"<<std::endl;
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
    }

    //----------------------------------------------------------------------
    //---------------------Transform our handles----------------------------
    //----------------------------------------------------------------------

    std::cout<<"here 2"<<std::endl;
    if(numSets>0){

        std::cout<<"why am I here?"<<std::endl;
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

    std::cout<<"here 3"<<std::endl;
    // 6. compute the deformation
    /// just as he says lets compute our deformation
    Eigen::SparseMatrix<double> A(m_laplaceMatrix);
    Eigen::SparseMatrix<double> b(m_deltaMatrix);
    //now lets solve it
    Eigen::SparseMatrix<double> AtA = A.transpose() * A;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::Upper > solver(AtA);
    Eigen::SparseMatrix<double> Atb = A.transpose() * b;

    Eigen::SparseMatrix<double> deformedPoints = solver.solve(Atb);
    if(solver.info()!=Eigen::Success){
        stat.perror("ERROR: Something when wrong with the solver. Soz about that :( <3");
        return MS::kFailure;
    }

    //----------------------------------------------------------------------
    //---------------------Set our new postions-----------------------------
    //----------------------------------------------------------------------

    std::cout<<"here 4"<<std::endl;
	// 7. set back result
    /// sets the new points of our mesh
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
    // lets declare that our eigen laplace matrix has not been created yet
    m_laplaceMatrixInit = false;
	// This sample creates a single input float attribute and a single
	// output float attribute.
	//
	MFnTypedAttribute	localCoordAttr;
	MFnMatrixAttribute	handleMatrixAttr;
	MFnTypedAttribute	associateAttr;
	MFnNumericAttribute	numSetsAttr;
    MFnNumericAttribute handlesAddedAttr;
	MStatus				stat;

    /// the full name and brief name are strings that can be used in maya scripting
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

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
    // Firstly we want any changes to our handle transforms to change our mesh to deform it
    stat = attributeAffects( m_handleTransformMatrix, outputGeom );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
    // Secondly we want any changes to our associations to be recomputed
    // e.g. when we change which handles our vertex's are associated to
    stat = attributeAffects( m_association, outputGeom );
    if (!stat) { stat.perror("attributeAffects"); return stat;}
    // Thirdly any changes in our local coordiates
    // This we mainly just be called when setting up the node
    stat = attributeAffects( m_localCoord, outputGeom );
    if (!stat) { stat.perror("attributeAffects"); return stat;}



	return MS::kSuccess;
}

