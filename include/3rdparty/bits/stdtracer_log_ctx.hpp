#pragma once
#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>
#include <stack>
#include <string>

#include <unistd.h>

#include "stdtracer_base.hpp"

struct xterm_t {
    const bool is_tty;
    xterm_t(uint8_t b, uint8_t f) : is_tty(isatty(fileno(stdout)))
    {
        if (is_tty) { printf("\e[%u;%um", b, f); }
    }

    ~xterm_t()
    {
        if (is_tty) { printf("\e[m"); }
    }
};

#define WITH_XTERM(b, f, e)                                                    \
    {                                                                          \
        xterm_t _(b, f);                                                       \
        e;                                                                     \
    }

template <typename clock_t, typename duration_t> class log_tracer_ctx_t_
{
  public:
    explicit log_tracer_ctx_t_(const std::string &name)
        : name(name), t0(clock_t::now()), depth(0)
    {
        log_files.push_front(stdout);
    }

    ~log_tracer_ctx_t_() {}

    void in(const std::string &name)
    {
        indent();
        WITH_XTERM(1, 35, printf("{ // [%s]", name.c_str()));
        putchar('\n');
        ++depth;
    }

    void out(const std::string &name, const duration_t &d)
    {
        --depth;

        char buffer[128];
        sprintf(buffer, "[%s] took ", name.c_str());
        [&](double t) {
            if (t < 1) {
                sprintf(buffer + strlen(buffer), "%.2fms", t * 1000);
            } else {
                sprintf(buffer + strlen(buffer), "%.2fs", t);
            }
        }(d.count());
        indent();
        WITH_XTERM(1, 32, printf("} // %s", buffer));
        putchar('\n');
    }

    void indent(FILE *fp = stdout)
    {
        for (int i = 0; i < depth; ++i) { fprintf(fp, "    "); }
    }

    template <typename... Args> void logf1(FILE *fp, const Args &... args)
    {
        fprintf(fp, "// ");
        fprintf(fp, args...);
        fputc('\n', fp);
    }

    template <typename... Args> void logf(const Args &... args)
    {
        for (auto fp : log_files) {
            if (fp == stdout) { indent(fp); }
            logf1(fp, args...);
            break;  // only log to the first
        }
    }

    std::deque<FILE *> log_files;

  private:
    const std::string name;
    const std::chrono::time_point<clock_t> t0;

    int depth;
};
