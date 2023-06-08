#pragma once

#include_next <fenv.h>

#ifndef HAVE_FEENABLEEXCEPT

inline int feenableexcept(int excepts)
{
    #pragma STDC FENV_ACCESS ON
    fexcept_t flags;
    /* Save current exception flags. */
    fegetexceptflag(&flags, FE_ALL_EXCEPT);

    feclearexcept(FE_ALL_EXCEPT);   /* clear all fp exception conditions */
    return fesetexceptflag(&flags, excepts) != 0 ? -1 : flags; /* set new flags */

}

inline int fedisableexcept(int excepts)
{
    #pragma STDC FENV_ACCESS ON
    fexcept_t flags;
    /* Save current exception flags. */
    fegetexceptflag(&flags, FE_ALL_EXCEPT);

    feclearexcept(FE_ALL_EXCEPT);   /* clear all fp exception conditions */
    return fesetexceptflag(&flags, ~excepts) != 0 ? -1 : flags; /* set new flags */
}

#endif // HAVE_FEENABLEEXCEPT
