#include <benchmark/benchmark.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/component/visual/InteractiveCamera.h>
#include <sofa/pointcloud/components/selectors/PointCloudOctreeSelector.h>
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/helper/logging/MessageHandler.h>
#include <sofa/type/Vec.h>
#include <fstream>
#include <random>
#include <limits>

using namespace sofa::pointcloud::components;
using sofa::component::visual::BaseCamera;
using sofa::component::visual::InteractiveCamera;
using sofa::core::objectmodel::New;

class OctreeFixture : public benchmark::Fixture {
public:
    PointCloudOctreeSelector* selector = nullptr;
    New<InteractiveCamera> camera;
    PointCloudContainer* container = nullptr;
    
    int maxSplats = 1000; 

    // S'exécute avant chaque test
    void SetUp(const ::benchmark::State& state) override {
        if (state.range_x() > 0) {
            maxSplats = static_cast<int>(state.range(0));
        }

        container = new PointCloudContainer();
        container->d_filename.setValue("splats/truck.ply");
        container->d_printLodding.setValue(false);
        container->init();

        selector = new PointCloudOctreeSelector();
        selector->l_geometries.add(container);
        selector->d_maxSplats.setValue(maxSplats);
        selector->initOctree();

        camera = New<InteractiveCamera>();
        camera->d_position.setValue(sofa::type::Vec3(0, 0, 10));
        camera->d_orientation.setValue(sofa::type::Quat(0, 0, 0, 1));
        camera->d_fieldOfView.setValue(45.0f);
        camera->d_zNear.setValue(0.1f);
        camera->d_zFar.setValue(1000.0f);        
        camera->d_distance.setValue(0); 
        camera->init();
    }

    // S'exécute après chaque test
    void TearDown(const ::benchmark::State& state) override {
        delete selector;
        delete container;
        selector = nullptr;
        container = nullptr;
    }
};

BENCHMARK_DEFINE_F(OctreeFixture, query_static)(benchmark::State& st) {
    std::vector<int> results;
    bool toggle = false;

    size_t min_splats = std::numeric_limits<size_t>::max();
    size_t max_splats_detected = 0;
    uint64_t total_splats = 0;
    uint64_t total_queries = 0;

    for (auto _ : st) {
        results.clear();

        camera->d_position.setValue(sofa::type::Vec3(0, 0, toggle ? 10.0001f : 10.0f));
        toggle = !toggle;

        selector->updateSelection(camera.get(), 1.33f, results);
        benchmark::DoNotOptimize(results);

        size_t current_splats = results.size();
        if (current_splats < min_splats) min_splats = current_splats;
        if (current_splats > max_splats_detected) max_splats_detected = current_splats;
        total_splats += current_splats;
        total_queries++;
    }

    if (total_queries > 0) {
        if (min_splats == std::numeric_limits<size_t>::max()) min_splats = 0;
        st.counters["OctreeSize"] = benchmark::Counter(static_cast<double>(selector->size()));
        st.counters["MinSplats"] = benchmark::Counter(static_cast<double>(min_splats));
        st.counters["MaxSplats"] = benchmark::Counter(static_cast<double>(max_splats_detected));
        st.counters["AvgSplats"] = benchmark::Counter(static_cast<double>(total_splats) / total_queries);
    }
}

BENCHMARK_DEFINE_F(OctreeFixture, query_approach_recede)(benchmark::State& st) {
    std::vector<int> results;
    double step = 0.0;
    const double dt = 0.01;

    size_t min_splats = std::numeric_limits<size_t>::max();
    size_t max_splats_detected = 0;
    uint64_t total_splats = 0;
    uint64_t total_queries = 0;

    float default_z = camera->d_position.getValue().z();

    for (auto _ : st) {
        results.clear();

        float z = default_z + 25.0f * std::sin(step);
        sofa::type::Vec3 position(0.0f, 0.0f, z);
        camera->d_position.setValue(position);

        selector->updateSelection(camera.get(), 1.33f, results);
        benchmark::DoNotOptimize(results);

        size_t current_splats = results.size();
        if (current_splats < min_splats) min_splats = current_splats;
        if (current_splats > max_splats_detected) max_splats_detected = current_splats;
        total_splats += current_splats;
        total_queries++;

        step += dt;
    }

    if (total_queries > 0) {
        if (min_splats == std::numeric_limits<size_t>::max()) min_splats = 0;
        st.counters["OctreeSize"] = benchmark::Counter(static_cast<double>(selector->size()));
        st.counters["MinSplats"] = benchmark::Counter(static_cast<double>(min_splats));
        st.counters["MaxSplats"] = benchmark::Counter(static_cast<double>(max_splats_detected));
        st.counters["AvgSplats"] = benchmark::Counter(static_cast<double>(total_splats) / total_queries);
    }
}

BENCHMARK_DEFINE_F(OctreeFixture, query_up_down)(benchmark::State& st) {
    std::vector<int> results;
    double step = 0.0;
    const double dt = 0.01;

    size_t min_splats = std::numeric_limits<size_t>::max();
    size_t max_splats_detected = 0;
    uint64_t total_splats = 0;
    uint64_t total_queries = 0;

    float default_y = camera->d_position.getValue().y();

    for (auto _ : st) {
        results.clear();

        float y = default_y + 25.0f * std::sin(step);
        sofa::type::Vec3 position(0.0f, y, 25.0f);
        camera->d_position.setValue(position);

        selector->updateSelection(camera.get(), 1.33f, results);
        benchmark::DoNotOptimize(results);

        size_t current_splats = results.size();
        if (current_splats < min_splats) min_splats = current_splats;
        if (current_splats > max_splats_detected) max_splats_detected = current_splats;
        total_splats += current_splats;
        total_queries++;

        step += dt;
    }

    if (total_queries > 0) {
        if (min_splats == std::numeric_limits<size_t>::max()) min_splats = 0;
        st.counters["OctreeSize"] = benchmark::Counter(static_cast<double>(selector->size()));
        st.counters["MinSplats"] = benchmark::Counter(static_cast<double>(min_splats));
        st.counters["MaxSplats"] = benchmark::Counter(static_cast<double>(max_splats_detected));
        st.counters["AvgSplats"] = benchmark::Counter(static_cast<double>(total_splats) / total_queries);
    }
}

BENCHMARK_DEFINE_F(OctreeFixture, query_left_right)(benchmark::State& st) {
    std::vector<int> results;
    double step = 0.0;
    const double dt = 0.01;

    size_t min_splats = std::numeric_limits<size_t>::max();
    size_t max_splats_detected = 0;
    uint64_t total_splats = 0;
    uint64_t total_queries = 0;

    float default_x = camera->d_position.getValue().x();

    for (auto _ : st) {
        results.clear();

        float x = default_x + 25.0f * std::sin(step);
        sofa::type::Vec3 position(x, 0.0f, 25.0f);
        camera->d_position.setValue(position);

        selector->updateSelection(camera.get(), 1.33f, results);
        benchmark::DoNotOptimize(results);

        size_t current_splats = results.size();
        if (current_splats < min_splats) min_splats = current_splats;
        if (current_splats > max_splats_detected) max_splats_detected = current_splats;
        total_splats += current_splats;
        total_queries++;

        step += dt;
    }

    if (total_queries > 0) {
        if (min_splats == std::numeric_limits<size_t>::max()) min_splats = 0;
        st.counters["OctreeSize"] = benchmark::Counter(static_cast<double>(selector->size()));
        st.counters["MinSplats"] = benchmark::Counter(static_cast<double>(min_splats));
        st.counters["MaxSplats"] = benchmark::Counter(static_cast<double>(max_splats_detected));
        st.counters["AvgSplats"] = benchmark::Counter(static_cast<double>(total_splats) / total_queries);
    }
}

BENCHMARK_REGISTER_F(OctreeFixture, query_static)->Arg(1000)->Arg(5000)->Arg(10000)->Arg(20000);
BENCHMARK_REGISTER_F(OctreeFixture, query_approach_recede)->Arg(1000)->Arg(5000)->Arg(10000)->Arg(20000);
BENCHMARK_REGISTER_F(OctreeFixture, query_up_down)->Arg(1000)->Arg(5000)->Arg(10000)->Arg(20000);
BENCHMARK_REGISTER_F(OctreeFixture, query_left_right)->Arg(1000)->Arg(5000)->Arg(10000)->Arg(20000);


BENCHMARK_MAIN();
