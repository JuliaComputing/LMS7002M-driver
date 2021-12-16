########################################################################
## LitePCIe discovery
########################################################################
find_path(LITEPCIE_KERNEL_INCLUDE_DIR litepcie.h
          REQUIRED
          HINTS
            ENV LITEPCIE_ROOT
          PATH_SUFFIXES
            kernel
            driver/kernel
            litepcie/software/kernel)
find_path(LITEPCIE_USER_INCLUDE_DIR liblitepcie.h
          REQUIRED
          HINTS
            ENV LITEPCIE_ROOT
          PATH_SUFFIXES
            user/liblitepcie
            driver/user/liblitepcie
            litepcie/software/user/liblitepcie)
set(LITEPCIE_INCLUDE_DIR ${LITEPCIE_KERNEL_INCLUDE_DIR} ${LITEPCIE_USER_INCLUDE_DIR})
find_library(LITEPCIE_LIBRARY litepcie
             REQUIRED
             HINTS
               ENV LITEPCIE_ROOT
             PATH_SUFFIXES
               user/liblitepcie
               driver/user/liblitepcie
               litepcie/software/user/liblitepcie)

########################################################################
## Main executable
########################################################################

add_executable(xtrx_main main.c)

target_include_directories(xtrx_main PRIVATE "../include")
target_include_directories(xtrx_main PRIVATE "../interfaces")
target_include_directories(xtrx_main PRIVATE ${LITEPCIE_INCLUDE_DIR})

target_link_libraries(xtrx_main ${LITEPCIE_LIBRARY} LMS7002M m)
