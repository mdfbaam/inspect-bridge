#pragma once

// C++
#include <string>

#ifndef _MSC_VER
#include <cxxabi.h>
#endif

namespace ornl::ros::ib::util {
    /*!
     * \brief Demangles a type name by using the underlying demangle routine from the respective libc++
     * \param type_name: Name of type to demangle.
     * \return Demangled name.
     * \note If you are confused about what this means, \see https://gcc.gnu.org/onlinedocs/libstdc++/manual/ext_demangling.html
     */
    inline std::string demangle(std::string type_name) {
#ifdef _MSC_VER
        return type_name;
#else
        int status = -1;
        char* demangled = abi::__cxa_demangle(type_name.c_str(), nullptr, nullptr, &status);

        if (status != 0) {
            free(demangled);

            return type_name;
        }

        std::string result(demangled);
        free(demangled);

        return result;
#endif
    }

    template<typename mangled_t>
    inline std::string demangle() {
        return demangle(typeid(mangled_t).name());
    }
}
