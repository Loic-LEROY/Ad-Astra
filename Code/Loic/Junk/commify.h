#ifndef COMMIFY_H
#define COMMIFY_H

#include <iostream>
#include <sstream>
#include <iomanip>

class Commify {
private:
    int64_t value;

public:
    explicit Commify(int64_t value) : value(value) {}

    friend std::ostream& operator<<(std::ostream &os, const Commify &c) {
        std::ostringstream oss;
        oss.imbue(std::locale(""));
        oss << std::fixed << c.value;
        os << oss.str();
        return os;
    }
};

#endif // COMMIFY_H