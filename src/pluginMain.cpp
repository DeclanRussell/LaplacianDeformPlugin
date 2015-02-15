//
// Copyright (C) Xiaosong Yang
// 
// File: pluginMain.cpp
//
// Author: Maya Plug-in Wizard 2.0
//

#include "LaplacianMeshDeformerNode.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{ 
	MStatus   status;
    MFnPlugin plugin( obj, "Declan Russell", "2015", "Any");

    status = plugin.registerNode( "LaplacianMeshDeformer", LaplacianMeshDeformer::m_id, LaplacianMeshDeformer::creator,
								  LaplacianMeshDeformer::initialize, MPxNode::kDeformerNode);
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj)
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

    status = plugin.deregisterNode( LaplacianMeshDeformer::m_id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
