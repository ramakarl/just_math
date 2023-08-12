
#ifndef DEF_MESH_INFO
	#define DEF_MESH_INFO

	typedef unsigned int	xref;

	#define MAX_MFORMAT		6
	#define MAX_BFORMAT		3

	// Edge attributes
	struct AttrEdge {
		xref	v1, v2, f1, f2;		// vertices and faces of an edge
	};	

	// Face attributes
	struct AttrV3 {
		xref	v1, v2, v3;			// vertices of face (tri)
	};
	struct AttrE3 {
		xref	e1, e2, e3;			// edges of face (tri)
	};
	struct AttrV4 {
		xref	v1, v2, v3, v4;		// verticies of face (quad)
	};
	struct AttrE4 {
		xref	e1, e2, e3, e4;		// edges of face (quad)
	};

	class MeshInfo {
	public:				
		enum MFormat {			// Mesh format
			UDef = 0,
			VV = 1,				//   Vertex-Vertex
			FV = 2,				//   Face-Vertex
			FVF = 3,
			WE = 4,				//   Winged-Edge
			CM = 5				//   Connected-Mesh
		};
		enum BFormat {			// Buffer format
			BVert = 0,
			BEdge = 1,
			BFace = 2,
		};
		enum AFormat {			// Extra Attribute formats
			APos = 0,
			AClr = 1,
			ANorm = 2,
			ATex = 3
		};
		//static int BufSize ( MFormat m, BFormat b )			{ return miBufSize[(int) m][(int) b]; }
		//static int miBufSize [MAX_MFORMAT][MAX_BFORMAT];
	};

#endif




