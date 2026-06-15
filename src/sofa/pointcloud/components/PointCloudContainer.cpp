/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2025 CNRS, INRIA, USTL, UJF, CNRS, MGH               *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/pointcloud/fwd.h>

#define TINYPLY_IMPLEMENTATION 1
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <Eigen/Geometry>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudContainer>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Store a point cloud.")
                             .add< sofa::pointcloud::components::PointCloudContainer >());
}

}

namespace sofa::core::objectmodel
{

/// Specialization for reading strings
template<>
bool Data<Eigen::MatrixXf>::read( const std::string& str ){}

/// Specialization for reading strings
template<>
std::string Data<Eigen::MatrixXf>::getValueString() const
{
    std::stringstream tmp;
    tmp << "EigenMatrix(" << m_value.getValue().rows() << "," << m_value.getValue().cols() << ")";
    return tmp.str();
}

}



namespace sofa::pointcloud::components
{

    PointCloudContainer::PointCloudContainer():
    d_filename(initData(&d_filename,"filename","Filename of the object")),
    d_printLodding(initData(&d_printLodding, true, "printLodding", "Print loading progress")),
    d_positions(initData(&d_positions, "positions", "Indices")), 
    d_orientations(initData(&d_orientations, "orientations", "Indices")), 
    d_scales(initData(&d_scales, "scales", "")), 
    d_opacities(initData(&d_opacities, "opacities", "")), 
    d_sphericalHarmonics(initData(&d_sphericalHarmonics, "sphericalHarmonics", "")), 
    d_indices(initData(&d_indices, "indices", "Indices")) {
        addUpdateCallback("updateBox", {&d_positions}, [this](const sofa::core::DataTracker& tracker)
        {
            updateBBox();
            return core::objectmodel::ComponentState::Valid;
        },{&f_bbox});
    }

    PointCloudContainer::~PointCloudContainer() {
        delete data;
    }


    void loader(std::atomic<bool>& running, const std::string& name) {
        std::vector<std::string> spinner = {"⠁","⠂","⠄","⡀","⢀","⠠","⠐","⠈"};
        int i = 0;

        std::cout << "\033[?25l"; // hide
        while (running) {
            std::cout << "\rLoading "<< name <<": " << spinner[i % spinner.size()] << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ++i;
        }
        std::cout << "\033[?25h"; // show
    }

    void PointCloudContainer::init()
    {
        if(isComponentStateValid())
            return;

        Inherit1::init();

        std::atomic<bool> running(true);
        std::thread t;
        if (d_printLodding.getValue())
            t = std::thread(loader, std::ref(running), d_filename.getValue());

        if(!d_filename.isSet() || d_filename.getValue() == ""){
            loadNaive();
            return;
        } else  if( !load(d_filename.getValue()) ){
            d_componentState = core::objectmodel::ComponentState::Invalid;
            running = false;
            if (d_printLodding.getValue())
                t.join();
            return;
        }

        running = false;
        if (d_printLodding.getValue())
            t.join();

        d_componentState = core::objectmodel::ComponentState::Valid;
    }

    void PointCloudContainer::updateBBox() {
        if(isComponentStateInvalid())
            return;

        type::BoundingBox box;
        auto points = data->xyz;
        for(int i=0;i<points.rows();++i)
        {
            auto point = points.row(i);
            box.include(type::Vec3{point.x(), point.y(), point.z()});
        }
        f_bbox.setValue(box);
    }

    void PointCloudContainer::loadNaive()
    {
        int maxSplats = 100;

        Eigen::MatrixXf gau_xyz(4, 3);  // Positions
        Eigen::MatrixXf gau_rot(4, 4);  // Rotations
        Eigen::MatrixXf gau_s(4, 3);    // Scales
        Eigen::MatrixXf gau_c(4, 3);    // Colors
        Eigen::MatrixXf gau_a(4, 1);    // Alpha ?
        for(int i = 0;i<maxSplats;i++)
        {
            gau_xyz << cos(i), sin(i), 0;
            gau_rot << 1, 0, 0, 0;
            gau_s << 0.03, 0.03, 0.03;

            gau_c << 1, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            gau_c = (gau_c.array() - 0.5f) / 0.28209f;

            gau_a << 1, 1, 1, 1;
        }

        data = new GaussianData{ gau_xyz, gau_rot, gau_s, gau_a, gau_c };
    };

    bool PointCloudContainer::load(const std::string& filename, int max_sh_degree) {
        std::string fullname = helper::system::DataRepository.getFile(filename);
        std::ifstream ss(fullname, std::ios::binary);
        std::string fname(filename);

        if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".ply") {
            msg_error() << "File is not a .ply file: " << fname;
            return false;
        }
        if (!ss.is_open()) {
            msg_error() << "Failed to open file: " << fname;
            return false;
        }

        PlyFile file;
        file.parse_header(ss);

        auto request = [&](const std::string& name) {
            return file.request_properties_from_element("vertex", { name }, 1);
        };

        auto x = request("x");
        auto y = request("y");
        auto z = request("z");
        auto opacity = request("opacity");
        auto rot_w = request("rot_0");
        auto rot_x = request("rot_1");
        auto rot_y = request("rot_2");
        auto rot_z = request("rot_3");
        auto scale_0 = request("scale_0");
        auto scale_1 = request("scale_1");
        auto scale_2 = request("scale_2");

        auto f_dc_0 = request("f_dc_0");
        auto f_dc_1 = request("f_dc_1");
        auto f_dc_2 = request("f_dc_2");

        int sh_coeffs = (max_sh_degree + 1) * (max_sh_degree + 1);
        std::vector<std::shared_ptr<PlyData>> f_rest_list;

        for (int i = 0; i < (sh_coeffs - 1); ++i) {
            f_rest_list.push_back(request("f_rest_" + std::to_string(i)));
            f_rest_list.push_back(request("f_rest_" + std::to_string(i + (sh_coeffs - 1))));
            f_rest_list.push_back(request("f_rest_" + std::to_string(i + 2 * (sh_coeffs - 1))));
        }

        file.read(ss);

        int N = x->count;
        auto load_vec = [](std::shared_ptr<PlyData>& pd, int N) -> Eigen::VectorXf {
            return Eigen::Map<Eigen::VectorXf>(reinterpret_cast<float*>(pd->buffer.get()), N);
        };

        Eigen::MatrixXf xyz(N, 3);
        xyz.col(0) = load_vec(x, N);
        xyz.col(1) = load_vec(y, N);
        xyz.col(2) = load_vec(z, N);

        Eigen::MatrixXf rot(N, 4);
        rot.col(0) = load_vec(rot_w, N);
        rot.col(1) = load_vec(rot_x, N);
        rot.col(2) = load_vec(rot_y, N);
        rot.col(3) = load_vec(rot_z, N);

        for (int i = 0; i < N; ++i) rot.row(i).normalize();

        Eigen::MatrixXf scale(N, 3);
        scale.col(0) = load_vec(scale_0, N).array().exp();
        scale.col(1) = load_vec(scale_1, N).array().exp();
        scale.col(2) = load_vec(scale_2, N).array().exp();

        Eigen::MatrixXf opac = load_vec(opacity, N);
        opac = (1.0f / (1.0f + (-opac.array()).exp())).eval();

        Eigen::MatrixXf sh(N, 3 * sh_coeffs);
        sh.col(0) = load_vec(f_dc_0, N);
        sh.col(1) = load_vec(f_dc_1, N);
        sh.col(2) = load_vec(f_dc_2, N);

        int num_extra = 3 * (sh_coeffs - 1);
        for (int i = 0; i < num_extra; ++i) {
            sh.col(3 + i) = load_vec(f_rest_list[i], N);
        }

        data = new GaussianData{ xyz, rot, scale, opac, sh };

        d_positions.setValue(xyz);
        d_orientations.setValue(rot);
        d_scales.setValue(scale);
        d_opacities.setValue(opac);
        d_sphericalHarmonics.setValue(sh);

        auto indices = helper::getWriteOnlyAccessor(d_indices);
        int maxSize = data->xyz.rows();
        indices.resize(maxSize);
        for(int i=0;i<maxSize;++i)
        {
            indices[i] = i;
        }
        return true;
    }

    size_t PointCloudContainer::size() {
        if(isComponentStateInvalid())
            return 0;

        return data->xyz.rows();
    }

    void PointCloudContainer::updateDataFields() {
        if(isComponentStateInvalid())
            return;

        d_positions.setValue(data->xyz);
        d_orientations.setValue(data->rot);
        d_scales.setValue(data->scale);
        d_componentState = core::objectmodel::ComponentState::Valid;
    }

}
