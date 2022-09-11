/*
 * The Mersenne Twister pseudo-random number generator (PRNG)
 *
 * This is an implementation of fast PRNG called MT19937, meaning it has a
 * period of 2^19937-1, which is a Mersenne prime.
 *
 * This PRNG is fast and suitable for non-cryptographic code.  For instance, it
 * would be perfect for Monte Carlo simulations, etc.
 *
 * For all the details on this algorithm, see the original paper:
 * http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/ARTICLES/mt.pdf
 *
 * Written by Christian Stigen Larsen
 * Distributed under the modified BSD license.
 * 2015-02-17, 2017-12-06
 * Edited mersenne twister to be a stateful class. 2007-2022 (c) Rama Hoetzlein, modified BSD license.
 */

#ifndef MERSENNE_TWISTER_H
	#define MERSENNE_TWISTER_H

	#include "common_defs.h"
	#include "vec.h"
	#include "quaternion.h"

	#define __STDC_LIMIT_MACROS
	#include <stdint.h>

	// We have an array of 624 32-bit values, and there are 31 unused bits, so we have a seed value of 624*32-31 = 19937 bits.
	static const size_t MT_SIZE = 624;
	static const size_t MT_PERIOD = 397;
	static const size_t MT_DIFF = MT_SIZE - MT_PERIOD;  // 227
	static const uint32_t MAGIC = 0x9908b0df;
	static const float I32_MAX = 4294967295.0f;
	static const float I32_HALFMAX = 2147483647.0f;

	 // State for a singleton Mersenne Twister. If you want to make this into a class, these are what you need to isolate
	class HELPAPI Mersenne {
	public:
		void generate_numbers();
		void seed(uint32_t val);

		uint32_t	randI();
		uint32_t	randI(int max);
		float		randF();			
		Vector3DF	randV3();
		Vector3DF	randV3(float vmin, float vmax);
		Quaternion	randQ();
	
	private:
		uint32_t MT[ MT_SIZE ];
		uint32_t MT_TEMPERED[ MT_SIZE];
		size_t index = MT_SIZE;
	};

#endif 