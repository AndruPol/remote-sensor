/*---------------------------------------------------------------------------*/
/* _printfs() - print formatted data to a file or a string                   */
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#define HUGE_VAL    1e256
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#ifdef __STDC__
#include <stdarg.h>
#else
#include <varargs.h>
#endif

#define POS_INFINITY  "inf"
#define NEG_INFINITY  "-inf"

#define _FLOAT_ROUND_ADJUST (double)5e-15


int vsprintf(char *string, const char *format, va_list ap);
int sprintf(char *buffer, char const *format, ...)
{
    int Len;
    va_list ap;
    va_start(ap, format);
    Len = vsprintf(buffer, format, ap);
    va_end(ap);
    return Len;
}

extern int _floatp10(double *fnum, uint_fast8_t *negative, int prec);

// as sizeof(int) == sizeof(long), 'l' specifier ignored
char *itoa(int num, char *str, int radix);
char *utoa(unsigned num, char *str, int radix);

int vsprintf(char *string, char const *format, va_list ap)
{
    char *output = string;
    char buffer[128];
    char *s;
    size_t width;
    uint_fast8_t prec;
    int fpower;
    uint_fast8_t negative;
    size_t padlen, zpadlen;
    double fnum;

    union
    {
        struct
        {
            uint_fast16_t    left_justify:1; // %-
            uint_fast16_t    show_sign:1;    // %+
            uint_fast16_t    zero_padding:1; // %0
            uint_fast16_t    pad_sign:1;     // %(space)
            uint_fast16_t    alt:1;          // %#
            uint_fast16_t    uppercase:1;    // %E%G%X / %e %g %x
//            uint_fast16_t    long_val:1;     // %l

            uint_fast16_t    precision:1;    // format string has precision field
        };
        uint_fast16_t    raw;
    } flags;

    for(; *format; format++)
    {
        if(*format != '%')
        {
            *output++ = *format;
        }
        else
        {
            flags.raw = 0;
            width = 0;
            prec = 0;
            padlen = 0;
            zpadlen = 0;
            s = buffer;

            while(*++format)
            {
                switch(*format)
                {
                case '-':
                    flags.left_justify = 1;
                    continue;
                case '+':
                    flags.show_sign = 1;
                    continue;
                case '0':
                    flags.zero_padding = 1;
                    continue;
                case ' ':
                    flags.pad_sign = 1;
                    continue;
                case '#':
                    flags.alt = 1;
                    continue;
                default:
                    break;
                }
                break;
            }
            if(*format == '*')
            {
                width = va_arg(ap, int);
                format++;
            }
            else
            {
                while(isdigit(*format))
                {
                    width = (width * 10) + (*format - '0');
                    format++;
                }
            }
            if(*format == '.')
            {
                format++;
                if(*format == '*')
                {
                    prec = va_arg(ap, unsigned int);
                    flags.precision = 1;
                    format++;
                }
                else
                {
                    while(isdigit(*format))
                    {
                        flags.precision = 1;
                        prec = (prec * 10) + (*format - '0');
                        format++;
                    }
                }
            }
            if(*format == 'l')
            {
 //               flags.long_val = 1;
                format++;
            }
            if(strchr("feEgG", *format))
            {
                if(fnum == HUGE_VAL)
                {
                    strcpy(s, POS_INFINITY);
                    flags.zero_padding = 0;
                    flags.precision = 0;
                    goto overflow;
                }
                else if(fnum == -HUGE_VAL)
                {
                    strcpy(s, NEG_INFINITY);
                    flags.zero_padding = 0;
                    flags.precision = 0;
                    goto overflow;
                }
            }
            switch(*format)
            {
            case 'c':
                *output++ = (char)va_arg(ap, int);
                continue;
            case 'd':
            case 'i':
//                if(flags.long_val)
//                    ltoa(va_arg(ap, long), buffer, 10);
//                else
                    itoa(va_arg(ap, int), buffer, 10);
                flags.alt = 0;
                break;
            case 'G':
                flags.uppercase = 1;
            case 'g':
                fnum = va_arg(ap, double);
                if(!flags.precision)
                    prec = 6;
                {
                    double sfnum = fnum;
                    fpower = _floatp10(&sfnum, &negative, -999);
                    sfnum = fnum;
                    fpower = _floatp10(&sfnum, &negative, prec - fpower);
                    if((fnum != 0.0) && ((fpower < -4) || (fpower >= (int)prec)))
                    {
                        fnum = sfnum;
                        goto format_Ee;
                    }
                }
                prec -= (fpower + 1);
                if((int)prec <= 0)
                    prec = 1;
                goto format_f;

            case 'E':
                flags.uppercase = 1;
            case 'e':
                fnum = va_arg(ap, double);
                if(!flags.precision)
                    prec = 6;
                {
                    double sfnum = fnum;
                    fpower = _floatp10(&sfnum, &negative, -999);
                    fpower = _floatp10(&fnum, &negative, prec - fpower);
                }
            format_Ee:
                if(negative)
                    *s++ = '-';
                {
                    int fdigit = (int)fnum;
                    *s++ = (char)(fdigit + '0');
                    fnum -= (double)fdigit;
                }
                fnum *= 10.0;
                fnum += _FLOAT_ROUND_ADJUST;
                if(flags.alt || prec)
                {
                    flags.precision = 0;
                    *s++ = '.';
                    uint_fast8_t fprec = 0;
                    while(prec)
                    {
                        if(fprec < 16)
                        {
                            int fdigit = (int)fnum;
                            *s++ = (char)(fdigit + '0');
                            fnum -= (double)fdigit;
                            fnum *= 10.0;
                            fnum += _FLOAT_ROUND_ADJUST;
                            fprec++;
                        }
                        else
                        {
                            *s++ = '0';
                        }
                        prec--;
                    }
                }
                *s = flags.uppercase ? 'E' : 'e';
                if(fpower >= 0)
                {
                    *s++ = '+';
                }
                else
                {
                    *s++ = '-';
                    fpower = -fpower;
                }
                if(fpower < 10)
                    *s++ = '0';
                itoa(fpower, s, 10);
                s = buffer;
                break;
            case 'f':
                fnum = va_arg(ap, double);
                if(!flags.precision)
                    prec = 6;
            format_f:
                fpower = _floatp10(&fnum, &negative, prec);
                if(negative)
                    *s++ = '-';
                if(fpower < 0)
                {
                    *s++ = '0';
                    fpower++;
                }
                else
                {
                    uint_fast8_t fprec = 0;
                    while(fpower >= 0)
                    {
                        if(fprec < 16)
                        {
                            int fdigit = (int)fnum;
                            *s++ = (char)(fdigit + '0');
                            fnum -= (double)fdigit;
                            fnum *= 10.0;
                            fnum += _FLOAT_ROUND_ADJUST;
                            fprec++;
                        }
                        else
                        {
                            *s++ = '0';
                        }
                        fpower--;
                    }
                    fpower = 0;
                }
                if(flags.alt || prec)
                {
                    flags.precision = 0;
                    *s++ = '.';
                    uint_fast8_t fprec = 0;
                    while(prec)
                    {
                        if(fpower < 0)
                        {
                            *s++ = '0';
                            fpower++;
                        }
                        else
                        {
                            if(fprec < 16)
                            {
                                int fdigit = (int)fnum;
                                *s++ = (char)(fdigit + '0');

                                fnum -= (double)fdigit;
                                fnum *= 10.0;
                                fnum += _FLOAT_ROUND_ADJUST;
                                fprec++;
                            }
                            else
                            {
                                *s++ = '0';
                            }
                        }
                        prec--;
                    }
                }
                *s = '\0';
                s = buffer;
                break;
            case 'n':
                *va_arg(ap, int *) = output - string;
                continue;

            case 'o':
                if(flags.alt)
                    *s++ = '0';
//                if(flags.long_val)
//                    ultoa(va_arg(ap, unsigned long), s, 8);
//                else
                    utoa(va_arg(ap, unsigned int), s, 8);
                s = buffer;
                break;

            case 'p':
                strupr(utoa((uintptr_t)va_arg(ap, void *), s, 16));
                break;

            case 's':
                s = va_arg(ap, char *);
                flags.show_sign = 0;
                flags.zero_padding = 0;
                flags.pad_sign = 0;
                break;

            case 'u':
//                if(flags.long_val)
//                    ultoa(va_arg(ap, unsigned long), s, 10);
//                else
                    utoa(va_arg(ap, unsigned int), s, 10);
                break;

            case 'x':
            case 'X':
                s = buffer;
                if(flags.alt)
                {
                    *s++ = '0';
                    *s++ = *format;
                }
//                if(flags.long_val)
//                    ultoa(va_arg(ap, unsigned long), s, 16);
//                else
                    utoa(va_arg(ap, unsigned int), s, 16);
                if(*format == 'X')
                    strupr(s);
                s = buffer;
                break;
            default:
                *output++ = *format;
                continue;
            }
        overflow:
            {
                size_t  ssize = strlen(s);
                size_t  dsize = ssize;
                char schar = '\0';
                size_t  maxlen = -1;

                if(flags.left_justify)
                    flags.zero_padding = 0;
                if(*format == 's')
                {
                    if(flags.precision)
                    {
                        flags.precision = 0;
                        maxlen = prec;
                    }
                }
                else
                {
                    if(ssize == 1 && *s == '0')
                    {
                        if(flags.precision && prec == 0)
                        {
                            *s = '\0';
                            ssize = 0;
                            dsize = 0;
                        }
                    }
                    if(*format == 'o' && flags.alt && prec < ssize) //????????
                    {
                        flags.precision = 1;
                        prec = ssize + 1;
                    }
                    if(*s == '-')
                    {
                        s++;
                        schar = '-';
                        dsize--;
                    }
                    else if(flags.show_sign)
                    {
                        schar = '+';
                        ssize++;
                    }
                    else if(flags.pad_sign)
                    {
                        schar = ' ';
                        ssize++;
                    }
                }
                if(width > ssize)
                {
                    if(flags.precision && (prec > dsize))
                    {
                        padlen = width - (ssize + (prec - dsize));
                    }
                    else
                    {
                        padlen = width - ssize;
                    }
                }
                if(flags.precision && (prec > dsize))
                {
                    zpadlen = prec - dsize;
                }
                else if(flags.zero_padding)
                {
                    zpadlen = padlen;
                    padlen = 0;
                }
                while(!flags.left_justify && (padlen > 0))
                {
                    *output++ = ' ';
                    padlen--;
                }

                if(schar)
                {
                    *output++ = schar;
                }

                while(zpadlen > 0)
                {
                    *output++ = '0';
                    zpadlen--;
                }
                while(*s)
                {
                    if(maxlen != (size_t)-1)
                    {
                        if(!(maxlen--)) break;
                    }
                    *output++ = *s++;
                }

                while(flags.left_justify && (padlen > 0))
                {
                    *output++ = ' ';
                    padlen--;
                }
            }
        }
    }
    *output = '\0';
    return(output - string);
}
