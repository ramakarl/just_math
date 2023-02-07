//--------------------------------------------------------------------------------
// JUST MATH:
// Artificial Neural Network 
//
// This code implements a simple Artificial Neural Network (ANN) 
// with backpropagation over scalar values (doubles).
// 
// Flexibility allows the number of layers and units to be easily specified
// with repeated calls to AddLayer, with input connections automatically created.
// For example, create a 1x5x5x1 network as follows:
//
//     AddLayer ( 0, 1 );        // input layer: 1 unit - first layer spec must have 0 in
//     AddLayer ( 1, 5 );        // 1 input connection , 5 units
//     AddLayer ( 5, 5 );        // 5 input connections, 5 units - fully connected layer
//     AddLayer ( 5, 1 );        // out layer: 5 inputs, 1 output unit
//
// The generation of training and test instances is separate from this class,
// allowing the to caller decide what kind of problem to learn. Caller uses the 
// FeedForward, Backprop and Retrieve functions to push instances thru the network, 
// to train data, and to retrieve output values.
//
// Efficiency is achieved with an offset and index storage scheme for the edges,
// so that the inner loop can sum a unit over edges using pointer increments.
// 
// Based on code I wrote from May 2000 for CS478, Machine Learning, at Cornell University.
// Rama Hoetzlein, 2023

//--------------------------------------------------------------------------------
// Copyright 2000, 2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
// * Derivative works may append the above copyright notice but should not remove or modify earlier notices.
//
// MIT License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction, including without
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#ifndef DEF_NEURAL_NET
#define DEF_NEURAL_NET

	#include <algorithm>		
	#include <stdlib.h>
	#include <string.h>
	#include <math.h>
	#include <vector>
	#include <string>

	#include "dataptr.h"	

	// Layer of network
    // - num_units = number of units in this layer
    // - num_in    = # of inputs per unit (this layer)
    // - total_in  = total # of inputs to this layer
    // - offs_u    = offset of first unit in layer
    // - offs_in   = offset of first edge in layer
	struct Layer {
		int num_units;		
		int num_in;
		int total_in;
		int offs_u;
		int offs_in;
	};

	// Unit of network
	// - id        = Unit ID (offset in buffer) of this unit
	// - layer     = Assigned layer
	// - w0        = Threshold weight (bias)
	// - out       = Unit output value (feed fwd)
	// - error     = Unit error value (backprop)
	struct Unit {
		int id;
		int layer;				
		double w0;
		double out;
		double error;
	};

	// Edge
	// - in_u     = Unit ID of input
	// - out_u    = Unit ID of output
	// - w        = Edge weight
	// - dw       = Edge weight momentum
	struct Edge {
		int in_u;
		int out_u;
		double w;
		double dw;
	};

	// Neural Network

	class NeuralNet {
	public:
		NeuralNet();

		// Build & Run
		int AddLayer( int ins, int units );
		void Connect ( int edge_id, int in_unit, int out_unit );
		void Initialize ( double wgt_rand, double rate, double momentum );
		void FeedForward ( DataPtr ins );
		void Backprop ( DataPtr outs );
		void Retrieve ( DataPtr outs );	

		void Print ();

		// Access functions
		int getNumLayers()		{ return mLayers.getNum(); }
		Layer* getLayer(int l)	{ return (Layer*) mLayers.getPtr(l); }
		
		int getNumUnits()		{ return mUnits.getNum(); }
		int getNumUnits(int l)  { return ((Layer*) mLayers.getPtr(l))->num_units; }
		Unit* getUnit( int id ) { return (Unit*) mUnits.getPtr(id); }
		Unit* getUnit( int l, int k ) {	return (Unit*) mUnits.getPtr( getUnitID(l,k) ); }
		int getUnitID ( int l, int k ) {
			int ou = ((Layer*) mLayers.getPtr(l))->offs_u;
			return ou + k;
		}		

		int getNumEdges()		{ return mEdges.getNum(); }
		int getNumEdges( int l ) { return ((Layer*) mLayers.getPtr(l))->num_in; }
		Edge* getEdge( int id )	{ return (Edge*) mEdges.getPtr(id); }
		Edge* getEdge( int l, int u, int ue) {			
			int oi = ((Layer*) mLayers.getPtr(l))->offs_in;
			int ni = ((Layer*) mLayers.getPtr(l))->num_in;
			int id = oi + (u*ni + ue);
			return (Edge*) mEdges.getPtr ( id );
		}

		// Neural Net buffers

		DataPtr		mLayers;
		DataPtr		mUnits;		
		DataPtr		mEdges;	

		float		learn_rate;
		float		learn_momentum;
		float		learn_wmax;
	};


#endif
