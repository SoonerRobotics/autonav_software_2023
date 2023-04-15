#pragma once

#include <string>
#include <stdint.h>

namespace Autonav
{
    int64_t hash(std::string str)
    {
        int64_t asd = 5381;
        int c;

        for (auto i = (size_t)0; i < str.length(); i++)
        {
            c = str[i];
            asd = ((asd << 5) + asd) + c; /* hash * 33 + c */
        }

        return asd & 0xFFFFFFFFFFFF;
    }
}