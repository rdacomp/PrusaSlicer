
#ifndef NahimicOSD_EXPORT_H
#define NahimicOSD_EXPORT_H

#ifdef NahimicOSD_BUILT_AS_STATIC
#  define NahimicOSD_EXPORT
#  define MYDLL_NO_EXPORT
#else
#  ifndef NahimicOSD_EXPORT
#    ifdef NahimicOSD_EXPORTS
        /* We are building this library */
#      define NahimicOSD_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define NahimicOSD_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef MYDLL_NO_EXPORT
#    define MYDLL_NO_EXPORT 
#  endif
#endif

#ifndef MYDLL_DEPRECATED
#  define MYDLL_DEPRECATED __declspec(deprecated)
#endif

#ifndef MYDLL_DEPRECATED_EXPORT
#  define MYDLL_DEPRECATED_EXPORT NahimicOSD_EXPORT MYDLL_DEPRECATED
#endif

#ifndef MYDLL_DEPRECATED_NO_EXPORT
#  define MYDLL_DEPRECATED_NO_EXPORT MYDLL_NO_EXPORT MYDLL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef MYDLL_NO_DEPRECATED
#    define MYDLL_NO_DEPRECATED
#  endif
#endif

#endif /* NahimicOSD_EXPORT_H */
