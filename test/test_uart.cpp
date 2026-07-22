// Host-side tests for the pure line-framing helpers in main/utils/uart.cpp.
// These need no ESP-IDF and no hardware:
//   c++ -std=c++17 -O0 -fsanitize=address,undefined test/test_uart.cpp main/utils/uart.cpp -Imain/utils -o /tmp/test_uart && /tmp/test_uart
#include "uart.h"
#include <cstring>
#include <iostream>
#include <string>

static int failures = 0;

static void expect_check(const std::string &line, const bool expected_ok, const std::string &expected_payload) {
    char buffer[512];
    std::strncpy(buffer, line.c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    bool ok = true;
    const int len = check(buffer, (int)line.size(), &ok);
    const std::string payload(buffer, len);
    const bool passed = ok == expected_ok && (!expected_ok || payload == expected_payload);
    std::cout << (passed ? "  ok   " : "  FAIL ") << '"' << line << "\" -> ok=" << ok
              << " payload=\"" << payload << "\"\n";
    if (!passed) {
        failures++;
    }
}

int main() {
    expect_check("foo@66", true, "foo");  // 'f' ^ 'o' ^ 'o' == 0x66
    expect_check("foo@00", false, "");    // parseable suffix, wrong checksum
    expect_check("foo@zz", false, "");    // unparseable suffix must not throw
    expect_check("foo@1z", false, "");    // std::stoi partial parse (0x01)
    expect_check("foo@ f", false, "");    // std::stoi skips leading whitespace
    expect_check("plain", true, "plain"); // no checksum suffix
    expect_check("@66", false, "");       // suffix only
    expect_check("", true, "");           // empty line
    std::cout << (failures == 0 ? "all vectors passed\n" : "FAILURES\n");
    return failures == 0 ? 0 : 1;
}
