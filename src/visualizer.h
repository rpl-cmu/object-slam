/******************************************************************************
 * File:             visualizer.h
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Visualizer thread
 *****************************************************************************/
#ifndef OSLAM_VISUALIZER_H
#define OSLAM_VISUALIZER_H

#include <Open3D/Open3D.h>
#include <mutex>
#include <thread>
#include <memory>

namespace oslam {
class Visualizer
{
  public:
    static Visualizer& get_instance()
    {
        static std::unique_ptr<Visualizer> instance(new Visualizer);
        return *instance;
    }
    Visualizer(const Visualizer &) = delete;
    Visualizer& operator=(const Visualizer &) = delete;
    virtual ~Visualizer();

    bool start(void);

    void add_geometry(std::shared_ptr<open3d::geometry::Geometry> p_geometry);
    void remove_geometry(std::shared_ptr<open3d::geometry::Geometry> p_geometry);
  private:
    explicit Visualizer();

    void run(void);
    /* data */
    std::unique_ptr<open3d::visualization::Visualizer> mp_visualizer;
    std::unique_ptr<std::thread> mp_thread;
    std::mutex m_mutex;

};
} /* namespace oslam */
#endif /* ifndef OSLAM_VISUALIZER_H */
