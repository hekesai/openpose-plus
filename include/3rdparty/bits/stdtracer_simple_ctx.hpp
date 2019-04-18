#pragma once
#include <algorithm>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "stdtracer_base.hpp"

template <typename clock_t, typename duration_t> class simple_tracer_ctx_t_
{
  public:
    explicit simple_tracer_ctx_t_(const std::string &name)
        : name(name), t0(clock_t::now()), depth(0)
    {
    }

    ~simple_tracer_ctx_t_()
    {
        if (!call_info_map.empty()) {
            constexpr const char *filename = "trace.log";
            fprintf(stderr, "// profile info logged to file://%s\n", filename);
            FILE *fp = fopen(filename, "w");
            report(fp);
            fclose(fp);
            report(stdout);
        }
    }

    void in(const std::string &name) { ++depth; }

    void out(const std::string &name, const duration_t &d)
    {
        auto &info = call_info_map[name];
        ++info.first;
        info.second += d;

        --depth;
    }

  private:
    const std::string name;
    const std::chrono::time_point<clock_t> t0;

    int depth;

    using call_info_t = std::pair<uint32_t, duration_t>;
    std::unordered_map<std::string, call_info_t> call_info_map;

    void report(FILE *fp) const
    {
        const auto total = since<double, clock_t>(t0);
        using item_t = std::tuple<duration_t, uint32_t, std::string>;
        std::vector<item_t> list;
        // for (const auto [name, duration] : total_durations) {
        for (const auto &it : call_info_map) {
            const auto name = it.first;
            const auto call_times = it.second.first;
            const auto duration = it.second.second;
            list.push_back(item_t(duration, call_times, name));
        }
        std::sort(list.rbegin(), list.rend());

        const std::string hr(80, '-');
        fprintf(fp, "\tsummary of %s::%s (%fs)\n", "tracer_ctx_t",  //
                name.c_str(), total.count());
        fprintf(fp, "%s\n", hr.c_str());
        fprintf(fp, "%8s    %16s    %12s    %12s    %s\n",  //
                "count", "cumulative (s)", "%", "mean (ms)", "call site");
        fprintf(fp, "%s\n", hr.c_str());
        // for (const auto &[duration, count, name] : list) {
        for (const auto &it : list) {
            const auto duration = std::get<0>(it);
            const auto count = std::get<1>(it);
            const auto name = std::get<2>(it);
            fprintf(fp, "%8d    %16f    %12.2f    %12.4f    %s\n",  //
                    count, duration.count(), duration * 100 / total,
                    1000 * duration.count() / count, name.c_str());
        }
    }
};
