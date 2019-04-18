#pragma once
#include <chrono>
#include <string>

#include "stdtracer_base.hpp"

template <typename ctx_t, typename clock_t = std::chrono::high_resolution_clock>
class scope_t_
{
  public:
    scope_t_(const std::string &name, ctx_t &ctx)
        : name(name), t0(clock_t::now()), ctx(ctx)
    {
        ctx.in(name);
    }

    ~scope_t_() { ctx.out(name, since<double, clock_t>(t0)); }

  private:
    const std::string name;
    const std::chrono::time_point<clock_t> t0;
    ctx_t &ctx;
};

template <typename clock_t,
          // typename... ctx_t,
          typename ctx_t_1, typename ctx_t_2>
class multi_ctx_scope_t_
{
  public:
    multi_ctx_scope_t_(const std::string &name,
                       //  ctx_t &... ctxs,
                       ctx_t_1 &c1, ctx_t_2 &c2)
        : name(name), t0(clock_t::now()),  // ctxs({ctxs...})
          c1(c1), c2(c2)
    {
        // {ctxs.in(name)...};
        c1.in(name);
        c2.in(name);
    }

    ~multi_ctx_scope_t_()
    {
        const auto d = since<double, clock_t>(t0);
        c2.out(name, d);
        c1.out(name, d);
        // ctxs.out(name, d);
    }

  private:
    const std::string name;
    const std::chrono::time_point<clock_t> t0;
    // ctx_t &ctx;
    // std::tuple<ctx_t &...> ctxs;
    ctx_t_1 &c1;
    ctx_t_2 &c2;
};

template <typename log_ctx_t> class set_trace_log_t
{
  public:
    set_trace_log_t(const std::string &name, log_ctx_t &ctx, bool reuse = false)
        : name(name), ctx(ctx)
    {
        FILE *fp = reuse  //
                       ? std::fopen(name.c_str(), "a")
                       : std::fopen(name.c_str(), "w");
        ctx.log_files.push_front(fp);
        ctx.indent();
        ctx.logf1(stdout, "start logging to %s", name.c_str());
    }

    ~set_trace_log_t()
    {
        ctx.indent();
        ctx.logf1(stdout, "stop logging to file://%s", name.c_str());
        FILE *fp = ctx.log_files.front();
        ctx.log_files.pop_front();
        std::fclose(fp);
    }

  private:
    const std::string name;
    log_ctx_t &ctx;
};
