#include "bvh.hpp"
#include "intersection.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
    using Clock = std::chrono::steady_clock;

    struct Measurement
    {
        std::vector<bool> flags;
        double milliseconds = 0.0;
    };

    std::size_t ReadPositiveNumber(const char* text)
    {
        const std::string argument(text);
        std::size_t value = 0;
        const auto parsed = std::from_chars(argument.data(), argument.data() + argument.size(), value);
        if (parsed.ec != std::errc{} || parsed.ptr != argument.data() + argument.size() || value == 0)
            throw std::invalid_argument("Expected a positive integer");
        return value;
    }

    std::vector<triangles::Triangle3> MakeScene(const std::string& scenario, std::size_t count)
    {
        std::vector<triangles::Triangle3> scene;
        scene.reserve(count);
        double x = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            if (scenario == "separated" || (scenario == "paired" && i % 2 == 0))
                x += 4.0;
            scene.push_back({{triangles::Point3{x, 0.0, 0.0},
                              triangles::Point3{x + 1.0, 0.0, 0.0},
                              triangles::Point3{x, 1.0, 0.0}}});
        }
        return scene;
    }

    Measurement MeasureBruteForce(const std::vector<triangles::Triangle3>& scene)
    {
        const auto start = Clock::now();
        auto flags = triangles::FindIntersectionsBruteForce(scene);
        const auto end = Clock::now();
        return {std::move(flags), std::chrono::duration<double, std::milli>(end - start).count()};
    }

    Measurement MeasureBvh(const std::vector<triangles::Triangle3>& scene, triangles::BvhSearchStats& stats)
    {
        const auto start = Clock::now();
        auto flags = triangles::FindIntersectionsBvh(scene, stats);
        const auto end = Clock::now();
        return {std::move(flags), std::chrono::duration<double, std::milli>(end - start).count()};
    }

    double Median(std::vector<double> values)
    {
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        if (values.size() % 2 == 0)
            return (values[middle - 1] + values[middle]) / 2.0;
        return values[middle];
    }

    void Benchmark(const std::string& scenario, std::size_t count, std::size_t repeats)
    {
        const auto scene = MakeScene(scenario, count);
        triangles::BvhSearchStats stats;
        if (triangles::FindIntersectionsBruteForce(scene) != triangles::FindIntersectionsBvh(scene, stats))
            throw std::runtime_error("Warmup masks differ in " + scenario);

        std::vector<double> brute_times;
        std::vector<double> bvh_times;
        for (std::size_t repeat = 0; repeat < repeats; ++repeat)
        {
            Measurement brute;
            Measurement bvh;
            if (repeat % 2 == 0)
            {
                brute = MeasureBruteForce(scene);
                bvh = MeasureBvh(scene, stats);
            }
            else
            {
                bvh = MeasureBvh(scene, stats);
                brute = MeasureBruteForce(scene);
            }
            if (brute.flags != bvh.flags)
                throw std::runtime_error("Measured masks differ in " + scenario);
            brute_times.push_back(brute.milliseconds);
            bvh_times.push_back(bvh.milliseconds);
        }

        std::cout << scenario << ',' << count << ',' << repeats << ','
                  << count * (count - 1) / 2 << ',' << stats.candidate_pairs << ','
                  << stats.exact_checks << ',' << Median(brute_times) << ','
                  << Median(bvh_times) << ",true\n";
    }
}

int main(int argc, char** argv)
{
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--help")
        {
            std::cout << "Usage: bvh_benchmark [N [repeats]]\n"
                      << "Defaults: N=512, repeats=3. Limits: N<=4096, repeats<=9.\n"
                      << "CSV contains median wall times; BVH time includes building the tree.\n";
            return 0;
        }
        if (argc > 3)
            throw std::invalid_argument("Usage: bvh_benchmark [N [repeats]]");
        const std::size_t count = argc > 1 ? ReadPositiveNumber(argv[1]) : 512;
        const std::size_t repeats = argc > 2 ? ReadPositiveNumber(argv[2]) : 3;
        if (count > 4096 || repeats > 9)
            throw std::invalid_argument("Benchmark limits: N<=4096, repeats<=9");

        std::cout << "scenario,n,repeats,bruteforce_pairs,candidate_pairs,exact_checks,bruteforce_ms,bvh_ms,masks_equal\n"
                  << std::fixed << std::setprecision(6);
        for (const std::string scenario : {"separated", "paired", "dense"})
            Benchmark(scenario, count, repeats);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}
