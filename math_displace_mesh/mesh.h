

#ifndef DEF_SHAPE_MESH
	#define DEF_SHAPE_MESH

	#include <string>
	#include <vector>
	#include <map>
	#include "vec.h"	
	#include "mesh_info.h"
	#include "datax.h"
	
	#ifdef MESH_DEBUG						// Heap debugging for FVF meshes
		#define VERT_DELTA		10000
		#define EDGE_DELTA		20000
		#define FACE_DELTA		30000
	#else
		#define VERT_DELTA		0
		#define EDGE_DELTA		0
		#define FACE_DELTA		0
	#endif

	#define MF_UNDEF	0							
	#define MF_FV		1
	#define MF_FVF		2
	#define MF_CM		3

	#define BVERTPOS		0		// vertices
	#define BVERTNORM		1
	#define BVERTTEX		2
	#define BVERTCLR		3
	#define BVERTFLIST		4
	#define BVERTELIST		5
	#define BEDGES			6		// edges
	#define BFACEV3			7		// faces
	#define BFACEE3			8
	#define BFACEV4			9
	#define BFACEE4			10
	#define BMAX			12

	struct SHPlyProperty {							// PLY Format structures
		char						type;
		std::string					name;
	};
	struct SHPlyElement {
		int							num;
		char						type;		// 0 = vert, 1 = face
		std::vector<SHPlyProperty>	prop_list;
	};

	#define PLY_UINT			0					// PLY Format constants
	#define PLY_INT				1
	#define PLY_FLOAT			2
	#define PLY_LIST			3
	#define PLY_VERTS			4
	#define PLY_FACES			5	

	#define MESH_VERTS			0					// Vertex buffers
	#define MESH_POS			1
	#define MESH_CLR			2	
	#define MESH_NORM			3
	#define MESH_TEX			4	
	#define MESH_VFLIST			5
	#define MESH_VELIST			6

	#define MESH_FACES			7					// Face buffers
	#define MESH_FV3			8
	#define MESH_FE3			9

	#define MESH_EDGES			10					// Edge buffers
	#define MESH_EDGE			11	

	typedef unsigned int		CLRVAL;

	class MeshX : public DataX {
	public:
		MeshX ();
		~MeshX ();	

		bool Load ( std::string fname, float scal=1.0 );

		#ifdef USE_NVGUI
			void DrawNormals (float ln, Matrix4F& xform);
			void Sketch (int w, int h, Camera3D * cam);
		#endif
		

		// Generic creation functions
		void		Clear ();		
		void		Debug ();
		void		DebugHeap ();
		void		SetUVRes ( int u, int v );
		int			GetURes ()				{ return m_Ures; }
		int			GetVRes ()				{ return m_Vres; }
		int			GetNumVert ()			{ return GetNumElem(BVERTPOS); }
		int			GetNumFace3 ()			{ return GetNumElem(BFACEV3); }
		int			GetNumEdge ()			{ return GetNumElem(BEDGES); }
		xref		AddVert (float x, float y, float z )			{ return (this->*m_AddVertFunc) (x, y, z); }
		xref		AddVert ( Vector3DF vec )						{ return (this->*m_AddVertFunc) (vec.x, vec.y, vec.z); }
		xref		AddVertNorm ( Vector3DF vec );		
		xref		AddVertClr ( Vector4DF vec );
		xref		AddVertTex ( Vector2DF vec );		
		xref		AddFaceFast (xref v1, xref v2, xref v3 )			{ return (this->*m_AddFaceFast3Func) (v1, v2, v3); }
		xref		AddFaceFast (xref v1, xref v2, xref v3, xref v4 )	{ return (this->*m_AddFaceFast4Func) (v1, v2, v3, v4); }

		void		ComputeBounds (Vector3DF& bmin, Vector3DF& bmax, int vmin=0, int vmax=0);
		Vector3DF	NormalizeMesh ( float sz, Vector3DF& ctr, int vmin = 0, int vmax = 0);		

		//void		BindFormat ( bufPos b );
		void		SetFormatFunc ();
		void		ComputeNormals (bool flat=false);		// Compute normals
		void		FlipNormals ();
		void		Smooth ( int iter );					// Smooth mesh
		void		UVSphere ();							// Add spherical tex coords
		void		Measure ();							// Measure mesh storage
					
		// Accessor functions
		CLRVAL*		GetVertClr ( int n )				{ return (CLRVAL*)		GetElem(BVERTCLR, n);  }
		Vector3DF*	GetVertPos ( int n )				{ return (Vector3DF*)	GetElem(BVERTPOS, n);  }
		Vector3DF*	GetVertNorm ( int n )				{ return (Vector3DF*)	GetElem(BVERTNORM, n); }
		Vector2DF*	GetVertTex ( int n )				{ return (Vector2DF*)	GetElem(BVERTTEX, n);  }
		hList*		GetVertFList ( int n )				{ return (hList*)		GetElem(BVERTFLIST, n);}
		hList*		GetVertEList ( int n )				{ return (hList*)		GetElem(BVERTELIST, n);}
		AttrV3*		GetFace3 ( int n )					{ return (AttrV3*)		GetElem(BFACEV3, n);}
		AttrE3*		GetFace3Edge ( int n )				{ return (AttrE3*)		GetElem(BFACEE3, n);}
		AttrV4*		GetFace4 ( int n )					{ return (AttrV4*)		GetElem(BFACEV4, n);}
		AttrE4*		GetFace4Edge ( int n )				{ return (AttrE4*)		GetElem(BFACEE4, n);}
		AttrEdge*	GetEdge ( int n )					{ return (AttrEdge*)	GetElem(BEDGES, n); }
		void		SetVertPos ( int n, Vector3DF pos )		{ SetElemVec3 ( BVERTPOS, n, pos) ; }
		void		SetVertTex ( int n, float u, float v )	{ Vector2DF x(u,v); SetElemVec2 ( BVERTTEX, n, x ); }

		void		SetVertNorm ( int n, Vector3DF norm )	{ SetElemVec3 ( BVERTNORM, n, norm); }
		void		SetVertClr ( int n, CLRVAL clr )		{ SetElemClr ( BVERTCLR, n, clr ); } 
		void		SetFace3 ( int n, xref v1, xref v2, xref v3 )			{ AttrV3 f; f.v1=v1; f.v2=v2; f.v3=v3;				SetElem ( BFACEV3, n, &f); }
		void		SetFace4 ( int n, xref v1, xref v2, xref v3, xref v4 )	{ AttrV4 f; f.v1=v1; f.v2=v2; f.v3=v3; f.v4=v4;		SetElem ( BFACEV4, n, &f); }
		void		SetEdge ( int n, xref f1, xref f2, xref v1, xref v2 )	{ AttrEdge e; e.f1=f1; e.f2=f2; e.v1=v1; e.v2=v2;	SetElem ( BEDGES, n, &e); }		

		// Load PLY format				
		bool		LoadPly ( const char* fname, float s=1.0f );
		void		AddPlyElement ( char typ, int n );
		void		AddPlyProperty ( char typ, std::string name ); 
		void		LoadPlyVerts ();
		void		LoadPlyFaces ();
		int			FindPlyElem ( char typ );
		int			FindPlyProp ( int elem, std::string name );

		// Load OBJ format
		bool		LoadObj ( const char* fname, float s=1.0f );

		// FV - Face-Vertex Mesh		
		// * Simplest format. Faces reference verts.
		// Buffer 0: Verts (x,y,z)
		// Buffer 1: Faces (v1, v2, v3) -> verts		
		void		CreateFV ();
		void		ClearFV ();
		void		SetFuncFV ();
		xref		AddVertFV ( float x, float y, float z );
		xref		AddFaceFast3FV ( xref v1, xref v2, xref v3 );
		xref		AddFaceFast4FV ( xref v1, xref v2, xref v3, xref v4 );				
		void		DebugFV ();

		// FVF - Face-Vertex-Face Mesh
		// * Explicit storage of neighboring faces of a vertex. Uses heap to store vertex neighborhood.
		// Buffer 0: Verts (x,y,z))(f1,f2,..fn) -> faces
		// Buffer 1: Faces (v1, v2, v3) -> verts
		void		CreateFVF ();
		void		ClearFVF ();
		void		SetFuncFVF ();
		xref		AddVertFVF ( float x, float y, float z );
		xref		AddFaceFast3FVF ( xref v1, xref v2, xref v3 );
		xref		AddFaceFast4FVF ( xref v1, xref v2, xref v3, xref v4 );				
		void		DebugFVF ();
		
		// CM - Connected Mesh
		// * Explicit storage of edges. Uses heap to store vertex neighborhood of edges and faces.
		// Buffer 0: Verts (x,y,z)(e1,e2,..en)(f1,f2,..fn) -> edges, faces
		// Buffer 1: Edges (v1,v2)(f1,f2) -> verts, faces  
		// Buffer 2: Faces (v1,v2,v3,v4)(e1,e2,e3,e4) -> verts, edges
		void		CreateCM ();
		void		SetFuncCM ();
		xref		AddVertCM ( float x, float y, float z );
		xref		AddFaceFast3CM ( xref v1, xref v2, xref v3 );
		xref		AddFaceFast4CM ( xref v1, xref v2, xref v3, xref v4 );
		xref		AddEdgeCM ( xref v1, xref v2, xref face );
		xref		FindEdgeCM ( xref v1, xref v2 );
		void		DebugCM ();
		void		ClearCM ();

		// Function pointers
		xref (MeshX::*m_AddVertFunc) (float x, float y, float z);
		xref (MeshX::*m_AddFaceFast3Func) (xref v1, xref v2, xref v3);
		xref (MeshX::*m_AddFaceFast4Func) (xref v1, xref v2, xref v3, xref v4);
	
	protected:
	
		int				m_Format;
		int				m_Ures, m_Vres;
	
		// PLY loading
		std::vector< SHPlyElement* >	m_Ply;		
		int			m_PlyCurrElem;
	};

#endif

