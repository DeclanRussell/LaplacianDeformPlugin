#ifndef _LaplacianMeshDeformerNode
#define _LaplacianMeshDeformerNode

/// @class LaplacianMeshDeformer
/// @author Declan Russell
/// @date 05/02/2015
/// @brief This is a deformer node that computes laplacian mesh editing upon a mesh to be used with Maya
/// @version 1.0
/// @todo in the deform function when you calculate your neighbour weights the cotangent weights are calculated incorrectly
/// @todo At the moment you are using mean value coordinates you should fix the cotangent weights as they are better


#include <maya/MPxDeformerNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MTypeId.h> 
#include <eigen3/Eigen/Sparse>

 
class LaplacianMeshDeformer : public MPxDeformerNode
{
public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief our default constructor
    //----------------------------------------------------------------------------------------------------------------------
    LaplacianMeshDeformer();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default destructor overiding the inherited MPxDeformerNode
    //----------------------------------------------------------------------------------------------------------------------
    virtual ~LaplacianMeshDeformer();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief our deform function. This is the function that calcualtes our deformation and is called every time we
    /// @brief modify the mesh based on the input attributes
    /// @param _iter - An iterator to our input geomtry that we want to deform
    /// @param _data - object that provides access to the attributes for this node
    //----------------------------------------------------------------------------------------------------------------------
    virtual MStatus deform(MDataBlock& 		_data, MItGeometry& 	_iter,   const MMatrix& 	_mat,  unsigned int		_multiIndex);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method exists to give Maya a way to create new objects of this type.
    /// @return a new object of this type
    //----------------------------------------------------------------------------------------------------------------------
    static  void* creator();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief initialize function that is automatically called when the node is registered with maya.
    /// @brief It will initialize all the attribute dependencies for the node.
    /// @return MS::kSuccess
    /// @return MS::kFailure
    //----------------------------------------------------------------------------------------------------------------------
    static  MStatus initialize();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This method returns a the attribute to which an accessory shape is connected.
    /// @brief If the accessory shape is deleted, the deformer node will automatically be deleted.
    /// @brief This method is optional when inheriting from MPxDeformer
    /// @return m_handleTransformMatrix - our only connected attribute
    //----------------------------------------------------------------------------------------------------------------------
    virtual MObject& accessoryAttribute() const;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This method is called when the deformer is created by the
    /// @brief "deformer" command. You can add to the cmds in the MDagModifier
    /// @brief cmd in order to hook up any additional nodes that your node needs
    /// @brief to operate.
    /// @brief In this example, we create a locator and attach its matrix attribute
    /// @brief to the matrix input on the offset node. The locator is used to
    /// @brief set the direction and scale of the random field.
    /// @brief This method is optional when inheriting from MPxDeformer
    //----------------------------------------------------------------------------------------------------------------------
    virtual MStatus accessoryNodeSetup(MDagModifier& _cmd);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief a simple linear interpolation function
    /// @param _x - input value one
    /// @param _y - input Value two
    /// @param _a - ratio of interpolatio between _x and _y
    //----------------------------------------------------------------------------------------------------------------------
    static float lerp(float _x, float _y, float _a);
    //----------------------------------------------------------------------------------------------------------------------


public:

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This MObject is an attirbute of the node which can be accessed by maya which we will use to set up values
    /// @brief These will be the local coordinates of the mesh unchanged by the deformation.
    /// @brief These will be used to calucalate our deformed mesh with the transform of our handles
    /// @brief have been added to them
    //----------------------------------------------------------------------------------------------------------------------
    static  MObject m_localCoord;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This MObject is an attirbute of the node which can be accessed by maya which we will use to set up values
    /// @brief This array will store the index of the linked handles for each vertex of our mesh.
    /// @brief -1 Means that there will be no association to a handle (free vertex)
    //----------------------------------------------------------------------------------------------------------------------
    static	MObject m_association;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This MObject is an attirbute of the node which can be accessed by maya which we will use to set up values
    /// @brief This is an array to store the transformation matrix for all our handles
    //----------------------------------------------------------------------------------------------------------------------
    static	MObject m_handleTransformMatrix;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief This MObject is an attirbute of the node which can be accessed by maya which we will use to set up values
    /// @brief The number of sets of points. One set of points will be associated with one handle
    //----------------------------------------------------------------------------------------------------------------------
    static	MObject m_numSets;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief The typeid is a unique 32bit indentifier that describes this node.
    /// @brief It is used to save and retrieve nodes of this type from the binary
    /// @brief file format.  If it is not unique, it will cause file IO problems.
    //----------------------------------------------------------------------------------------------------------------------
    static	MTypeId m_id;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Eigen sparse matrix for our laplacian matrix. Sparse matricies very good for large meshes!
    //----------------------------------------------------------------------------------------------------------------------
    static Eigen::SparseMatrix<double> m_laplaceMatrix;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Eigen sparse matrix for our delta matrix
    /// @todo Not use sparse matix for this matrix as it will give you a performance hit
    //----------------------------------------------------------------------------------------------------------------------
    static Eigen::SparseMatrix<double> m_deltaMatrix;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief a bool to declare if our laplace matrix has been created yet
    //----------------------------------------------------------------------------------------------------------------------
    static bool m_laplaceMatrixInit;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief a member to declare if our handle have added and need updating
    //----------------------------------------------------------------------------------------------------------------------
    static MObject m_handlesAdded;
    //----------------------------------------------------------------------------------------------------------------------

};

#endif
