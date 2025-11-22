
#ifndef DEF_WANGTILES
	#define DEF_WANGTILES

	#include "vec.h"

	struct Tile
	{
		int n, e, s, w;
		int numSubtiles, numSubdivs, numPoints, numSubPoints;
		int ** subdivs;
		Vec2F *points;
		Vec2F *subPoints;
	};

	class WangTiles {
	public:
		WangTiles ();
		
		bool	LoadTileSet ( const char* fileName );
		void	SetDensityFunc ( float* density, int xres, int yres );
		void	SetMaxPoints (int m );

		int		RecurseTileImage ( Vec2F cmin, Vec2F cmax, float zm, float ts );
		void	RecurseTileImage (Tile & t, float x, float y, int level);

		int		numPnts ()				{ return mNumPnts; }
		Vec3F getPnt ( int n )	{ return mPoints[n]; }

	private:
		
		float		mZoom;
		Vec2F		mClipMin, mClipMax;

		float*		mDensity;				// input density function
		int			mXRes, mYRes;
	
		int			mNumPnts, mMaxPnts;		// output points 
		Vec3F*	mPoints;
		Vec3F*  mCurrPnt;

		Tile*		mTiles;					// wang tile data
		int			numTiles;
		int			numSubtiles;
		int			numSubdivs;

		float		toneScale;
	};

#endif


