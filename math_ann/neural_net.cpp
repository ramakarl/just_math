//--------------------------------------------------------------------------------
// JUST MATH:
// Artificial Neural Network 
//
//

//--------------------------------------------------------------------------------
// Copyright 2019-2023 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
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

#include <algorithm>
#include "dataptr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>

#include "neural_net.h"

NeuralNet::NeuralNet()
{
}

void NeuralNet::Connect ( int edge_id, int in_unit, int out_unit ) 
{
	Edge* e = getEdge(edge_id);
	e->in_u = in_unit;
	e->out_u = out_unit;
}


int NeuralNet::AddLayer ( int ins, int units )
{
	Layer lprev, lnew;
	Unit unit;

	// Get previous layer
	lprev.num_in = 0;
	lprev.num_units = 0;
	lprev.offs_u = 0;
	int L = mLayers.getNum();	
	if (L > 0) {
		lprev = * (Layer*) mLayers.getPtr(L-1);
	}

	// new layer
	int total_in = units * ins;		
	int start_unit = mUnits.getNum();

	mLayers.Append ( sizeof(Layer), 1, 0, DT_CPU );			mLayers.UseMax();
	mUnits.Append  ( sizeof(Unit), units, 0, DT_CPU );		mUnits.UseMax();
	mEdges.Append ( sizeof(Edge), total_in, 0, DT_CPU );	mEdges.UseMax();

	// Append edges
	// - make connections automatically by evenly distributing the 
	//   inputs (lprev) to new units based on ins (# inputs per unit)	
	int start_inp = mEdges.getNum() - total_in;	
	int prev_unit = lprev.offs_u;
	int unit_in, j=0;
	float du = float(lprev.num_units - ins) / float(ins-1);
	float ds = float(lprev.num_units - ins) / float(units-1);
	int su_max = lprev.num_units-1-(ins-1)*du;
	if (lprev.num_units < ins || ins==1 || du < 1) du = 1;	
	if (units==1) ds = 1;
	float su = 0;
	for (int i=0; i < units; i++) {
		for (int k=0; k < ins; k++) {
			unit_in = prev_unit + int(su) + int(k*du);
			Connect ( start_inp + j, unit_in, start_unit + i );	
			j++;
		}
		su += ds;
		if ( su > su_max) su = su_max;
		if ( su < 0 ) su = 0;
	}	

	// Append units
	for (int i=0; i < units; i++) {
		unit.id = start_unit + i;
		unit.layer = L;		
		mUnits.SetElem ( start_unit + i, &unit );
	}

	// Append layer		
	lnew.num_units = units;
	lnew.num_in = ins;
	lnew.total_in = total_in;
	lnew.offs_in = start_inp;
	lnew.offs_u = start_unit;
	mLayers.SetElem ( L, &lnew );

	return 1;
}

void NeuralNet::Initialize ( double wgt_rand, double rate, double momentum )
{
	learn_wmax = wgt_rand;
	learn_rate = rate;
	learn_momentum = momentum;

	// Initialize network with random weights and biases
	Unit* u;	
	for (int i=0; i < getNumUnits(); i++) {
		u = getUnit(i);
		u->out = 0;
		u->w0 = ((rand()*wgt_rand*2.0)/RAND_MAX) - wgt_rand;		
	}

	Edge* e;
	for (int j=0; j < getNumEdges(); j++) {
		e = getEdge(j);
		e->w = ((rand()*wgt_rand*2.0)/RAND_MAX) - wgt_rand;
		e->dw = 0;
	}
}

void NeuralNet::Print  ()
{
	char val[128];
	int width = 25;
	
	Edge* e;
	Layer* lr;
	int maxu = 0;
	for (int j=0; j < getNumLayers(); j++) {
		lr = getLayer(j);
		maxu = (lr->num_units > maxu) ? lr->num_units : maxu;
	}

	// Print heading	
	std::string col, msg;
	for (int l=0; l < getNumLayers(); l++) {
		sprintf ( val, "Layer %d", l );
		col = val;
		int csz = col.size();
		for (int a=0; a < width-csz; a++)  col += " ";	
		msg += col;
	}
	dbgprintf ( "%s\n", msg.c_str() );

	// Print units		
	
	int ui;

	for (int lu=0; lu <= maxu; lu++) {
		msg = "";
		for (int l=0; l < getNumLayers(); l++) {
			col = "";
			lr = getLayer(l);
			ui = getUnitID(l, lu);
			// print at this row and column if this layer has one
			if (lu < lr->num_units) {				
				// collect edges to identify incoming units
				for (int k=0; k < lr->num_in; k++) {
					e = getEdge ( l, lu, k );
					sprintf ( val, "%d", e->in_u );
					col = col + val;
					if ( k != lr->num_in-1) col += ",";
				}
				if (lr->num_in > 0 ) col += " -> ";				
				// print current unit
				sprintf( val, "%d (%3.3f)", ui, (float) getUnit(ui)->out );
				//sprintf( val, "%d", ui );
				col += val;
			}
			// pad output to columns
			int csz = col.size();
			for (int a=0; a < width-csz; a++)  col += " ";			
			msg = msg + col;		
		}
		// write the entire row
		dbgprintf ( "%s\n", msg.c_str() );
	}
}

void NeuralNet::FeedForward ( DataPtr ins  )
{
	Layer* lr;
	Unit *u, *in_unit;
	Edge* e;
	double sum;

	// Layer 0 - Write data to network inputs
	double* train_in = (double*) ins.getPtr(0);
	for (int i=0; i < getNumUnits(0); i++) {
		u = getUnit(0, i);
		u->out = *train_in;
		train_in++;
	}

	// Propogate inputs through network to output
	int num_layers = getNumLayers();
	for (int l=1; l < num_layers; l++) {
		lr = getLayer(l);
		
		// Process all units in the current layer
		for (int ui=0; ui < getNumUnits(l); ui++) {
			u = getUnit(l, ui);
			
			// Sum all inputs which feed to this unit			
			e = getEdge(l, ui, 0);

			sum = u->w0;
			for (int j = 0; j < getNumEdges(l); j++) {
				// sum = w0 + SUM[ edge weight * input unit value ]
				in_unit = getUnit( e->in_u );
				sum += in_unit->out * e->w;		
				e++;		
			}

			// Compute unit activation function
			if (l==num_layers-1) {
				// Linear activation
				u->out = sum;
			} else {
				// Sigmoid activation
				u->out = 1.0 / (1.0 + exp(-sum));
			}
		}
	}
}


void NeuralNet::Backprop ( DataPtr outs )
{
	Layer* lr;
	Unit *u, *in_unit;
	Edge* e;
	int li;
	int num_layers = getNumLayers();

	double* train_out = (double*) outs.getPtr(0);

	// Initialize all unit errors to zero		
	for (int i=0; i < getNumUnits(); i++) {
		u = getUnit(i);
		u->error = 0;		
	}
	
	// Propagate the errors backward through network
	
	// -- a) Process errors for last layer outputs
	li = num_layers-1;
	lr = getLayer( li );
	for (int ui=0; ui < getNumUnits( li ); ui++) {
		u = getUnit(li, ui);

		// Compute the output error as the difference between
		// training data and current output values
		u->error = *train_out - u->out;
		train_out++;

		// Transmit errors backwards to hidden units
		//  (this is the summation portion of step 3 in Mitchell's Backpropagation)
		// Sum all inputs which feed to this unit
		e = getEdge(li, ui, 0);		
		for (int j = 0; j < getNumEdges(li); j++) {
			in_unit = getUnit( e->in_u );
			in_unit->error += e->w * u->error;
			e++;
		}
	}
	
	// -- b) Process errors for hidden units
	for (li = num_layers-2; li >= 1; li-- ) {
		lr = getLayer( li );
		for (int ui=0; ui < getNumUnits( li ); ui++) {
			u = getUnit(li, ui);

			// Attenuate error based on output values
			u->error *= u->out * (1.0 - u->out);

			// Transmit errors backwards to hidden units
			//  (this is the summation portion of step 3 in Mitchell's Backpropagation)
			// Sum all inputs which feed to this unit
			e = getEdge(li, ui, 0);		
			for (int j = 0; j < getNumEdges(li); j++) {
				in_unit = getUnit( e->in_u );
				in_unit->error += e->w * u->error;
				e++;
			}
		}
	}

	
	double update;

	// -- c) Update network weights
	for (li = num_layers-1; li >= 1; li-- ) {
		lr = getLayer( li );
		for (int ui=0; ui < getNumUnits( li ); ui++) {
			u = getUnit(li, ui);

			// Update the weight bias
			u->w0 += learn_rate * u->error * 1.0;
			
			e = getEdge(li, ui, 0);		
			for (int j = 0; j < getNumEdges(li); j++) {
				in_unit = getUnit( e->in_u );

				// Update with standard weight changes
				update = learn_rate * u->error * in_unit->out;
				// Include momentum in weight changes
				update += learn_momentum * e->dw;
				// Update weights using weight changes
				e->w += update;
				// Update weight changes themselves
				e->dw = update;				
				e++;
			}
		}	
	}
}

void NeuralNet::Retrieve ( DataPtr outs )
{
	int li = getNumLayers()-1;
	Layer* lr = getLayer( li );
	Unit* u;

	// Collect output layer into out buffer
	double* vals_out = (double*) outs.getPtr(0);
	for (int ui=0; ui < getNumUnits( li ); ui++) {
		u = getUnit(li, ui);
		*vals_out = u->out;
		vals_out++;
	}
}
