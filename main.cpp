#include <threepp/threepp.hpp>
#include <vector>
#include <mutex>
#include <chrono>
#include "include/Sphero.h"
#include "include/SimulatorUtilities.h"
#include "include/KeyHandler.h"
#include "include/Communication.h"
#include "include/Slam.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <opencv2/opencv.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <string>
#include <utility>

using namespace threepp;
std::shared_ptr<Mesh> createBox2(const Vector3& pos, const Color& color) {
    auto geometry = BoxGeometry::create(0.90, 0.1, 0.90);
    auto material = MeshPhongMaterial::create();
    material->color = color;

    auto box = Mesh::create(geometry, material);
    box->position.copy(pos);

    return box;
}
#include <fstream> // For saving data
#include <iomanip> // For saving data
int main() {

    // Rest of the scene
    Clock clock;
    Canvas canvas{ "Sphero Simulator", {{"aa", 8}} };
    GLRenderer renderer(canvas.size());


    auto scene = Scene::create();
    auto camera = PerspectiveCamera::create(75, canvas.aspect(), 0.1f, 100);
    camera->position.set(0, 3, 5);
    camera->lookAt(0, 0, 0);
    OrbitControls controls(*camera, canvas);

    // Add lights
    auto ambientLight = AmbientLight::create(Color::white, 0.5f);
    scene->add(ambientLight);
    auto directionalLight = DirectionalLight::create(Color::white, 0.5f);
    directionalLight->position.set(0, 10, 10);
    scene->add(directionalLight);

    // Add boxes for testing lidar detection
    auto cube1 = createBox({ 2, 0, 3 }, Color::blue);
    auto cube2 = createBox({ 5, 0, -5 }, Color::red);
    auto cube3 = createBox({ -3, 0, 0 }, Color::green);
    auto cube4 = createBox({ 0.3, 0, 1 }, Color::blue);
    auto cube5 = createBox({ 1, 0, 5 }, Color::red);
    auto cube6 = createBox({ 4.4, 0, 4.2 }, Color::green);
    scene->add(cube1);
    scene->add(cube2);
    scene->add(cube3);
    scene->add(cube4);
    scene->add(cube5);
    scene->add(cube6);




    // Load additional objects
    OBJLoader loader;
    TextureLoader tl;

    auto tex = tl.load("Assets/Room_texture.png");
    auto obj2 = loader.load("Assets/The Test Room.obj", true);

    obj2->traverseType<Mesh>([tex](Mesh& child) {
        auto m = MeshPhongMaterial::create();
        m->map = tex;
        //m->side = threepp::Side::Double;  // Enable backface culling
        child.setMaterial(m);
    });
    obj2->position.set(0, 0, 0);
    scene->add(obj2);

    // Create Sphero object
    Sphero sphero(scene, "127.0.0.1", 65432);  // Provide the host and port
    sphero.position.y = 0.2;
    scene->add(sphero);
    sphero.setLidarSpeed(600.0);
    sphero.enableSweep(false);

    // Set up objects to scan
    std::vector<Object3D*> objectsToScan = { cube1.get(), cube2.get(), cube3.get(), cube4.get(), cube5.get(), cube6.get() };
    obj2->traverseType<Mesh>([&objectsToScan](Mesh& child) {
        child.geometry()->computeBoundingBox();
        objectsToScan.push_back(&child);
    });
    sphero.setScanObjects(objectsToScan);

    // Attach POV camera to Sphero
    auto povCamera = PerspectiveCamera::create(75, canvas.aspect(), 0.1f, 100);
    povCamera->position.set(0.2, 0.5, 0.0);
    povCamera->lookAt(10.0,0.5,0.0);
    sphero.add(povCamera);

    // Key controls setup
    KeyController keyController(sphero);
    canvas.addKeyListener(keyController);

    // Handle window resize
    canvas.onWindowResize([&](WindowSize size) {
        camera->aspect = size.aspect();
        camera->updateProjectionMatrix();
        renderer.setSize(size);
    });

    NewSlam new_slam(scene);


    float time = 0.0f;
    const float interval = 0.01f; //SLAMINTERVAL
    bool firstTime = true;
    // Render and display loop
    canvas.animate([&]() {
        float deltaTime = clock.getDelta();
        keyController.update(deltaTime);
        sphero.update(deltaTime);
        time += deltaTime;

        // Process visualization updates
        {
            std::lock_guard<std::mutex> lock(new_slam.myGridInstance.visualizationQueueMutex);
            while (!new_slam.myGridInstance.visualizationQueue.empty()) {
                new_slam.myGridInstance.visualizationQueue.front()(); // Execute the visualization update
                new_slam.myGridInstance.visualizationQueue.pop();
            }
        }
        // Add SLAM frame to slam
        if (time > interval) {
            auto data = sphero.getSLAMframe();
            if (!data.first.empty()) {
                new_slam.enqueueFrame(data);
            }
            time = 0.0f;
        }


        if (sphero.pov) {
            povCamera->fov = 100;
            povCamera->updateProjectionMatrix(); // to fix pov change
            renderer.render(*scene, *povCamera);
        } else {
            renderer.render(*scene, *camera);
        }

    });


    return 0;
}
