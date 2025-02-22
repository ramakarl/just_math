

#include "common_defs.h"

#include "wang_tiles.h"
#include "imagex.h"

#include <stdio.h>
#include <stdlib.h>

/*
	Copyright 2006 Johannes Kopf (kopf@inf.uni-konstanz.de)
	Implementation of the algorithms described in the paper

	Recursive Wang Tiles for Real-Time Blue Noise
	Johannes Kopf, Daniel Cohen-Or, Oliver Deussen, Dani Lischinski
	In ACM Transactions on Graphics 25, 3 (Proc. SIGGRAPH 2006)
*/

WangTiles::WangTiles ()
{		
	mDensity = 0x0;
	mPoints = 0x0;
	toneScale = 1.0;

	SetMaxPoints ( 1000000 );		// allocate output points
}

int freadi(FILE* fp)
{
	int v;
	fread(&v, sizeof(int), 1, fp );
	return v;	
}
float freadf(FILE* fp)
{
	float v;
	fread(&v, sizeof(float), 1, fp);
	return v;
}
int isqrt(int val)
{
	unsigned long temp, g = 0, b = 0x8000, bshft = 15;
	do {
		if (val >= (temp = (((g << 1) + b) << bshft--))) {
			g += b;
			val -= temp;
		}
	} while (b >>= 1);
	return g;
}

bool WangTiles::LoadTileSet(const char * fileName)
{
	FILE * fin = fopen(fileName, "rb");
	if ( fin==0x0 ) return false;
	
	numTiles = freadi(fin);
	numSubtiles = freadi(fin);
	numSubdivs = freadi(fin);

	mTiles = new Tile[numTiles];

	for (int i = 0; i < numTiles; i++)
	{
		mTiles[i].n = freadi(fin);
		mTiles[i].e = freadi(fin);
		mTiles[i].s = freadi(fin);
		mTiles[i].w = freadi(fin);

		mTiles[i].subdivs = new int * [numSubdivs];
		for (int j = 0; j < numSubdivs; j++) {
			int* subdiv = new int[numSubtiles * numSubtiles];
			for (int k = 0; k < numSubtiles * numSubtiles; k++)
				subdiv[k] = freadi(fin);
			mTiles[i].subdivs[j] = subdiv;
		}

		mTiles[i].numPoints = freadi(fin);		
		mTiles[i].points = new Vec2F [mTiles[i].numPoints];
		for (int j = 0; j < mTiles[i].numPoints; j++)
		{
			mTiles[i].points[j].x = freadf(fin);
			mTiles[i].points[j].y = freadf(fin);
			freadi(fin); freadi(fin); freadi(fin); freadi(fin);
		}

		mTiles[i].numSubPoints = freadi(fin);
		mTiles[i].subPoints = new Vec2F[mTiles[i].numSubPoints];
		for (int j = 0; j < mTiles[i].numSubPoints; j++)
		{
			mTiles[i].subPoints[j].x = freadf(fin);
			mTiles[i].subPoints[j].y = freadf(fin);
			freadi(fin); freadi(fin); freadi(fin); freadi(fin);
		}
		dbgprintf ( "Wang Tile: %d, subt: %d, pnts:%d, subpnt:%d\n", i, numSubtiles, mTiles[i].numPoints, mTiles[i].numSubPoints );
	}	
	fclose(fin);
	
	return true;
}

void WangTiles::SetMaxPoints ( int p )
{
	mMaxPnts = p;
	if (mPoints != 0) delete mPoints;
	mPoints = new Vec3F[mMaxPnts];
}

void WangTiles::SetDensityFunc(float* density, int xres, int yres)
{
	mDensity = density;
	mXRes = xres;
	mYRes = yres;
}

int WangTiles::RecurseTileImage ( Vec2F cmin, Vec2F cmax, float zm, float ts )
{
	if (mDensity==0x0) dbgprintf ( "ERROR: Wang tiles has no density image.\n" );
	if (mMaxPnts== 0) dbgprintf ("ERROR: Wang tiles has no max points.\n");	
	
	mClipMin = cmin;
	mClipMax = cmax;
	mZoom = 1.0 / zm;
	toneScale = ts * 256.0;

	mNumPnts = 0;
	mCurrPnt = mPoints;

	RecurseTileImage ( mTiles[0], 0, 0, 0);

	return mNumPnts;
}

void WangTiles::RecurseTileImage (Tile & t, float x, float y, int level )
{
	float tileSize = 1.f / powf(float(numSubtiles), float(level));
	
	if ((x+tileSize < mClipMin.x) || (x > mClipMax.x) || (y+tileSize < mClipMin.y) || (y > mClipMax.y))
		return;

	float factor = toneScale * tileSize*tileSize / mZoom;
	int tilePnts = imin(t.numSubPoints, int( factor - t.numPoints ) );
		
	if ( mNumPnts + tilePnts > mMaxPnts )
		return;

	// generate points
	float px, py, v;
	Vec3F* basepnt = mCurrPnt;

	for (int i = 0; i < tilePnts; i++)
	{
		px = x + t.subPoints[i].x*tileSize;
		py = y + t.subPoints[i].y*tileSize;
		// skip point if it lies outside of the clipping window
		if ((px < mClipMin.x) || (px > mClipMax.x) || (py < mClipMin.y) || (py > mClipMax.y))
			continue;

		// evaluate density function
		v = mDensity[int(py * mYRes)*mXRes + int(px*mXRes)];
		if ( v*v <= i / factor )
			continue;

		// output point			
		mCurrPnt->Set ( px, py, level );
		mCurrPnt++;
	}
	mNumPnts += (mCurrPnt - basepnt);

	// recursion
	if (factor - t.numPoints > t.numSubPoints) {
		for (int ty = 0; ty < numSubtiles; ty++)
			for (int tx = 0; tx < numSubtiles; tx++)
				RecurseTileImage(mTiles[t.subdivs[0][ty * numSubtiles + tx]], x + tx * tileSize / numSubtiles, y + ty * tileSize / numSubtiles, level + 1);
	}

}