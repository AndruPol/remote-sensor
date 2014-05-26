/*---------------------------------------------------------------------------*/
/* _floatp10() get the characteristic, mantissa and sign of a double         */
/*             precision floating point number such that:                    */
/*                                                                           */
/*                     1 <= mantissa < 10                                    */
/*              - 308 <= characteristic <= 308                               */
/*                                                                           */
/*            negative: false if positive                                    */
/*                      true  if negative                                    */
/*---------------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>

int _floatp10(double *fnum, uint_fast8_t *negative, int prec)
{
    int i;
    int fpower = 0;
    int ipower = 256;
    int rpower = 256;
    double fround = 0.5;

    static double const pfpower[] =
    {
      1.0e+256,
      1.0e+128,
      1.0e+64,
      1.0e+32,
      1.0e+16,
      1.0e+8,
      1.0e+4,
      1.0e+2,
      1.0e+1
    };

    static double const nfpower[] =
    {
      1.0e-256,
      1.0e-128,
      1.0e-64,
      1.0e-32,
      1.0e-16,
      1.0e-8,
      1.0e-4,
      1.0e-2,
      1.0e-1
    };

    *negative = *fnum < (double)0;

    if(*fnum != 0.0)
    {
        if(prec > 0)
        {
            if(prec < 309)
            {
                for(i = 0; i < 9; i++)
                {
                    if(prec >= rpower)
                    {
                        fround /= pfpower[i];
                        prec -= rpower;
                    }
                    rpower >>= 1;
                }
            }
            else
                fround = (double)0;
        }
        else if(prec < 0)
        {
            if(prec > -310)
            {
                fround = (double)5;
                for(i = 0; i < 9; i++)
                {
                    if(prec >= rpower)
                    {
                        fround *= pfpower[i];
                        prec += rpower;
                    }
                    rpower >>= 1;
                }
            }
            else
                fround = (double)0;
        }
        *fnum = fabs(*fnum) + fround;
        if(*fnum < (double)1)
        {
            for(i = 0; i < 9; i++)
            {
                if(*fnum <= nfpower[i])
                {
                    *fnum *= pfpower[i];
                    fpower -= ipower;
                }
                ipower >>= 1;
            }
        }
        else if(*fnum >= (double)10)
        {
            for(i = 0; i < 9; i++)
            {
                if(*fnum >= pfpower[i])
                {
                    *fnum /= pfpower[i];
                    fpower += ipower;
                }
                ipower >>= 1;
            }
        }
        if(*fnum < (double)1)
        {
            *fnum *= pfpower[8];
            fpower--;
        }
    }
    return fpower;
}
