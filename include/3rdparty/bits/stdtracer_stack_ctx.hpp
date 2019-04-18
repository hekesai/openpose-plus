#pragma once
#include <algorithm>
#include <chrono>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "stdtracer_base.hpp"

template <typename clock_t, typename duration_t> class stack_tracer_ctx_t_
{
  public:
    stack_tracer_ctx_t_(const std::string &name)
        : name(name), t0(clock_t::now()), call_stack_str("")
    {
    }

    ~stack_tracer_ctx_t_()
    {
        if (!call_info_map.empty()) { report(stdout); }
    }

    void in(const std::string &name)
    {
        const int idx = get_idx(name);
        call_stack_str += "/" + std::to_string(idx);
    }

    void out(const std::string &name, const duration_t &d)
    {
        auto &info = call_info_map[call_stack_str];
        ++info.first;
        info.second += d;

        call_stack_str.erase(call_stack_str.rfind('/'));
    }

    void report(FILE *fp)
    {
        const auto total = since<double>(t0);
        using item_t = std::tuple<duration_t, uint32_t, std::string>;
        std::vector<item_t> list;
        // for (const auto [name, duration] : total_durations) {
        for (const auto &it : call_info_map) {
            const auto name = it.first;
            const auto info = it.second;
            list.push_back(item_t(info.second, info.first, name));
        }
        std::sort(list.rbegin(), list.rend());

        const std::string hr(80, '-');
        fprintf(fp, "\tinvoke tree of %s::%s (%fs)\n", "stack_tracer_ctx_t_",
                name.c_str(), total.count());
        fprintf(fp, "%s\n", hr.c_str());
        fprintf(fp, "%8s    %16s    %12s    %12s    %s\n",  //
                "count", "cumulative (s)", "%", "mean (ms)", "call chain");
        fprintf(fp, "%s\n", hr.c_str());
        // for (const auto &[duration, count, name] : list) {
        for (const auto &it : list) {
            const auto duration = std::get<0>(it);
            const auto count = std::get<1>(it);
            const auto name = decode_call_stack_str(std::get<2>(it));
            fprintf(fp, "%8d    %16f    %12.2f    %12.4f    %s\n",  //
                    count, duration.count(), duration * 100 / total,
                    1000 * duration.count() / count, name.c_str());
        }
    }

  private:
    const std::string name;
    const std::chrono::time_point<clock_t> t0;

    std::string call_stack_str;

    std::unordered_map<std::string, int> index;
    std::vector<std::string> names;

    using call_info_t = std::pair<uint32_t, duration_t>;
    std::unordered_map<std::string, call_info_t> call_info_map;

    int get_idx(const std::string &name)
    {
        const auto pos = index.find(name);
        if (pos != index.end()) { return pos->second; }
        const int idx = names.size();
        names.push_back(name);
        index[name] = idx;
        return idx;
    }

    static std::vector<std::string> split(const std::string &text,
                                          const char sep)
    {
        std::vector<std::string> lines;
        std::string line;
        std::istringstream ss(text);
        while (std::getline(ss, line, sep)) {
            if (!line.empty()) { lines.push_back(line); }
        }
        return lines;
    }

    std::string decode_call_stack_str(const std::string &call_ss)
    {
        std::string ss;
        for (const auto &s : split(call_ss, '/')) {
            const int idx = std::stoi(s);
            ss += "/" + names[idx];
        }
        return ss;
    }
};
