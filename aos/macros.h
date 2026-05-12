#ifndef _AOS_MACROS_H_
#define _AOS_MACROS_H_

// This file has some common macros etc.

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName &) = delete;     \
  void operator=(const TypeName &) = delete

// AOS_OS_NONE is defined to 1 if we are building for a platform without an OS
// (like a microcontroller), and 0 otherwise. This is managed via the
// //aos:macros target in Bazel.
#ifndef AOS_OS_NONE
#error "AOS_OS_NONE must be defined. Depend on //aos:macros."
#endif

#define AOS_STRINGIFY(x) AOS_TO_STRING(x)
#define AOS_TO_STRING(x) #x

#ifdef __clang__
#define GOOD_PRINTF_FORMAT_TYPE __printf__
#else
#define GOOD_PRINTF_FORMAT_TYPE gnu_printf
#endif

#endif  // _AOS_MACROS_H_
