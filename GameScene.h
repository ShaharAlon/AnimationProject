#pragma once
#include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "CamModel.h"
#include "../../build/tutorial/Assignment3/Saves.h"
#include <AABB.h>


class GameScene : public cg3d::SceneWithImGui
{
public:
    GameScene(std::string name, cg3d::Display* display);
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void AddViewportCallback(cg3d::Viewport* _viewport) override;
    void StartGui();
    void PlayGui();
    void ViewportSizeCallback(cg3d::Viewport* _viewport) override;

private:
    typedef
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;

    typedef struct Price {
        int type;
        std::shared_ptr<cg3d::Model> model;
        igl::AABB<Eigen::MatrixXd, 3> tree;
        Eigen::Vector3d dir;
    };
    enum class States { PLAY, PAUSE, MENU,LOSE, WIN };

    void BuildImGui() override;
    inline bool IsActive() const { return state==States::PLAY; };
    void LoadObjectFromFileDialog();
    void SetCamera(int index);
    static std::shared_ptr<CamModel> CreateCameraWithModel(int width, int height, float fov, float near, float far, const std::shared_ptr<cg3d::Material>& material);
    static void DumpMeshData(const Eigen::IOFormat& simple, const cg3d::MeshData& data) ;
    void restartLevel();
    void initLevel();
    void levelUp();
    void gameTick();
    void priceEffect(int type);
    bool checkCollision(igl::AABB<Eigen::MatrixXd, 3>* t1, igl::AABB<Eigen::MatrixXd, 3>* t2);
    bool checkBoxInter(Eigen::AlignedBox<double, 3> b1, Eigen::AlignedBox<double, 3> b2);
    void checkSnakeCollision();
    void movePrices();
    void moveSnake();
    void moveVertices();
    void createW();
    Eigen::VectorXd createWi(double w1, int idx_w1, double w2, int idx_w2);
    Eigen::Vector3d getRandomDir();
    Eigen::Vector3d getRandomLocation();
    void addPrices(int type, int amount);
    Eigen::Vector3d position_offset;
    Eigen::Vector3f dir = { 0.001,0,0 };
    float velo = 0.001;
    Eigen::MatrixXd V, W, C, U, M;
    Eigen::MatrixXi F, BE;
    Eigen::VectorXi P;
    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;
    std::vector<Eigen::Vector3d> skeleton;
    int tick = 0;
    int level = 1;
    int progress = 0;
    int score = 0;
    int health = 30;
    States state = States::MENU;
    char pName[256] = "";
    std::shared_ptr<Movable> root;
    std::shared_ptr<Movable> pricesRoot;
    std::shared_ptr<cg3d::Material> carbon;
    std::vector<std::shared_ptr<cg3d::Camera>> camList{4};
    cg3d::Viewport* viewport = nullptr;
    Saves* saves = new Saves();
    bool show = false, nameErr = false;
    std::shared_ptr<cg3d::AutoMorphingModel> snake;
    std::vector<std::shared_ptr<cg3d::Model>> joints;
    std::vector < std::shared_ptr<cg3d::Material>> mats;
    std::shared_ptr<cg3d::Model> check1, check2;
    std::vector<igl::AABB<Eigen::MatrixXd, 3>> trees;
    igl::AABB<Eigen::MatrixXd, 3> sTree;
    std::vector<Price> prices;


};

