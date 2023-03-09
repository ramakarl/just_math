

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

#include <stdio.h>
#include "mersenne.h"

 // Better on older Intel Core i7, but worse on newer Intel Xeon CPUs (undefine
 // it on those).
 //#define MT_UNROLL_MORE
#define M32(x) (0x80000000 & x) // 32nd MSB
#define L31(x) (0x7FFFFFFF & x) // 31 LSBs

#define UNROLL(expr) \
  y = M32( MT[i]) | L31( MT[i+1]); \
  MT[i] = MT[expr] ^ (y >> 1) ^ (((int32_t(y) << 31) >> 31) & MAGIC); \
  ++i;

void Mersenne::generate_numbers()
{
    // For performance reasons, we've unrolled the loop three times, thus mitigating the need for any modulus operations. 
    // Anyway, it seems this trick is old hat: http://www.quadibloc.com/crypto/co4814.htm
    size_t i = 0;
    uint32_t y;
    
    while (i < MT_DIFF) {      // i = [0 ... 226]
        // We're doing 226 = 113*2, an even number of steps, so we can safely unroll one more step here for speed:
        UNROLL(i + MT_PERIOD);
        #ifdef MT_UNROLL_MORE
            UNROLL(i + MT_PERIOD);
        #endif
    }    
    while (i < MT_SIZE - 1) {      // i = [227 ... 622]
        // 623-227 = 396 = 2*2*3*3*11, so we can unroll this loop in any number that evenly divides 396 (2, 4, 6, etc). Here we'll unroll 11 times.        
        UNROLL(i - MT_DIFF);
        #ifdef MT_UNROLL_MORE
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
            UNROLL(i - DIFF);
        #endif
    }
    {
        // i = 623, last step rolls over
        y = M32( MT[ MT_SIZE - 1]) | L31( MT[0]);
        MT[ MT_SIZE - 1] = MT[ MT_PERIOD - 1] ^ (y >> 1) ^ (((int32_t(y) << 31) >> 31) & MAGIC);
    }

    // Temper all numbers in a batch
    for (size_t i = 0; i < MT_SIZE; ++i) {
        y = MT[i];
        y ^= y >> 11;
        y ^= y << 7 & 0x9d2c5680;
        y ^= y << 15 & 0xefc60000;
        y ^= y >> 18;
        MT_TEMPERED[i] = y;
    }
    index = 0;
}

void Mersenne::seed (uint32_t value)
{
    // The equation below is a linear congruential generator (LCG), one of the oldest known pseudo-random number generator algorithms, in the form
    //    X_(n+1) = = (a*X_n + c) (mod m).
    // We've implicitly got m=32 (mask + word size of 32 bits), so there is no need to explicitly use modulus.
    // What is interesting is the multiplier a.  The one we have below is 0x6c07865 --- 1812433253 in decimal, and is called the Borosh-Niederreiter
    // multiplier for modulus 2^32.
    // See: OPTIMAL MULTIPLIERS FOR PSEUDO-RANDOM NUMBER GENERATION BY THE LINEAR CONGRUENTIAL METHOD (1983) at http://www.springerlink.com/content/n7765ku70w8857l7/
    //      LCGs at http://en.wikipedia.org/wiki/Linear_congruential_generator . Says: "A common Mersenne twister implementation, interestingly enough, uses an LCG to generate seed data.",
    // Since we're using 32-bits data types for our MT array, we can skip the masking with 0xFFFFFFFF below.
    MT[0] = value;
    index = MT_SIZE;
    for (uint_fast32_t i = 1; i < MT_SIZE; ++i)
        MT[i] = 0x6c078965 * (MT[i - 1] ^ MT[i - 1] >> 30) + i;

    generate_numbers();
}

uint32_t Mersenne::randI ()
{
    if (index == MT_SIZE) {
        generate_numbers();
        index = 0;
    }
    return MT_TEMPERED[ index++ ];
}
uint32_t Mersenne::randI(int max)
{
    return (uint64_t(randI()) * uint64_t(max)) / I32_MAX;
}
uint32_t Mersenne::randI(int vmin, int vmax)
{
    return vmin + (uint64_t(randI()) * uint64_t(vmax-vmin)) / I32_MAX;
}
float Mersenne::randF()
{
    // range: 0 to 1
    return float(randI()) / I32_MAX;
}
float Mersenne::randF(float vmin, float vmax)
{
    return vmin + float(randI() * (vmax - vmin)) / I32_MAX;
}
Vector3DF Mersenne::randV3()
{
    // range: 0 to 1
    return Vector3DF( float(randI())/ I32_MAX,
                      float(randI())/ I32_MAX,
                      float(randI())/ I32_MAX );
}
Vector3DF Mersenne::randV3(float vmin, float vmax)
{
    float a = vmin + float(randI() * (vmax - vmin)) / I32_MAX;
    float b = vmin + float(randI() * (vmax - vmin)) / I32_MAX;
    float c = vmin + float(randI() * (vmax - vmin)) / I32_MAX;
    return Vector3DF(a, b, c );
}
Vector3DF Mersenne::randV3(Vector3DF vmin, Vector3DF vmax)
{
    float a = vmin.x + float(randI() * (vmax.x - vmin.x)) / I32_MAX;
    float b = vmin.y + float(randI() * (vmax.y - vmin.y)) / I32_MAX;
    float c = vmin.z + float(randI() * (vmax.z - vmin.z)) / I32_MAX;
    return Vector3DF(a, b, c );
}
Vector4DF Mersenne::randV4(float vmin, float vmax)
{
    return Vector4DF(vmin + float(randI() * (vmax - vmin)) / I32_MAX,
                    vmin + float(randI() * (vmax - vmin)) / I32_MAX,
                    vmin + float(randI() * (vmax - vmin)) / I32_MAX,
                    vmin + float(randI() * (vmax - vmin)) / I32_MAX);
}
Vector4DF Mersenne::randV4(Vector4DF vmin, Vector4DF vmax)
{
    return Vector4DF(vmin.x + float(randI() * (vmax.x - vmin.x)) / I32_MAX,
                    vmin.y + float(randI() * (vmax.y - vmin.y)) / I32_MAX,
                    vmin.z + float(randI() * (vmax.z - vmin.z)) / I32_MAX,
                    vmin.w + float(randI() * (vmax.w - vmin.w)) / I32_MAX);
}


Quaternion Mersenne::randQ()
{
    Quaternion q;
    q.set ( float(randI()) / I32_HALFMAX - 1.0f,
            float(randI()) / I32_HALFMAX - 1.0f,
            float(randI()) / I32_HALFMAX - 1.0f,
            float(randI()) / I32_HALFMAX - 1.0f );
    q.normalize();
    return q;
}