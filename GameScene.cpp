#include "GameScene.h"

#include <utility>
#include "ObjLoader.h"
#include "SceneWithImGui.h"
#include "CamModel.h"
#include "Visitor.h"
#include "Utility.h"
#include "IglMeshLoader.h"

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include <random>
#include <dqs.h>


using namespace cg3d;

void GameScene::BuildImGui()
{
    switch (state) {
        case States::MENU:
            StartGui();
            break;
        case States::PLAY:
            PlayGui();
            break;
        case States::PAUSE:
            PlayGui();
            break;
        case States::WIN:
            StartGui();
            break;
        case States::LOSE:
            StartGui();
            break;

    }
}

void GameScene::DumpMeshData(const Eigen::IOFormat& simple, const MeshData& data)
{
    std::cout << "vertices mesh: " << data.vertices.format(simple) << std::endl;
    std::cout << "faces mesh: " << data.faces.format(simple) << std::endl;
    std::cout << "vertex normals mesh: " << data.vertexNormals.format(simple) << std::endl;
    std::cout << "texture coordinates mesh: " << data.textureCoords.format(simple) << std::endl;
}

void GameScene::restartLevel()
{
    tick = 0;
    health = 30;
    velo = 0.001;
    dir = { 0.001,0,0 };
    for (int i = 0; i < prices.size(); i++) {
        prices[i].model->Translate(-prices[i].model->GetTranslation());
        prices[i].model->Translate(getRandomLocation().cast<float>());
    }
    snake->Translate(-snake->GetTranslation());
}

void GameScene::initLevel()
{
    addPrices(3, level);
    for (int i = 0; i < 3; i++) {
        addPrices(i, 2 + level);
    }
}

void GameScene::levelUp()
{
    for (int i = 0; i < 4; i++) {
        addPrices(i, 1);
    }
    for (int i = 0; i < prices.size(); i++) {
        prices[i].model->Translate(-prices[i].model->GetTranslation());
        prices[i].model->Translate(getRandomLocation().cast<float>());
    }
    snake->Translate(-snake->GetTranslation());
}

void GameScene::gameTick()
{
    if (IsActive()) {
        tick++;
        //moveSnake();
        snake->Translate(dir);
        movePrices();
        checkSnakeCollision();
    }
    if (tick > 100000 || health < 1) {
        state = States::LOSE;
        level = 0;
        restartLevel();
    }
    if (progress == level) {
        state = States::PAUSE;
        level++;
        progress = 0;
        levelUp();
    }
    if (level == 6)
    {
        state = States::WIN;
        saves->saveScore(pName, score);
        level = 1;
        score = 0;
        restartLevel();
    }
}

void GameScene::priceEffect(int type)
{
    switch (type) {
        case 0:
            health -= 10;
            score--;
            break;
        case 1:
            velo += 0.001;
            score++;
            break;
        case 2:
            health += 5;
            score++;
            break;
        case 3:
            progress++;
            score += 5;
            break;
        
    }
}

bool GameScene::checkCollision(igl::AABB<Eigen::MatrixXd, 3>* t1, igl::AABB<Eigen::MatrixXd, 3>* t2)
{

    if (!checkBoxInter(t1->m_box, t2->m_box)) {
        return false;
    }
    return true;
 /*   if (t1->is_leaf() && t2->is_leaf()) {
        if (checkBoxInter(t1->m_box, t2->m_box))
            return true;
        else
            return false;
    }
    if (t1->is_leaf())
        return checkCollision(t1, t2->m_right) || checkCollision(t1, t2->m_left);

    if (t2->is_leaf())
        return checkCollision(t1->m_right, t2) || checkCollision(t1->m_left, t2);

    return checkCollision(t1->m_right, t2->m_left) || checkCollision(t1->m_left, t2->m_right) || checkCollision(t1->m_right, t2->m_right) || checkCollision(t1->m_left, t2->m_left);*/
}

bool GameScene::checkBoxInter(Eigen::AlignedBox<double, 3> b1, Eigen::AlignedBox<double, 3> b2)
{
    Eigen::Matrix3d A = check1->GetRotation().cast<double>().transpose();
    Eigen::Matrix3d B = check2->GetRotation().cast<double>().transpose();
    Eigen::Matrix3d C = A.transpose() * B;
    Eigen::Matrix4f Scale1 = check1->GetScaling(check1->GetTransform()).matrix();
    Eigen::Matrix4f Scale2 = check2->GetScaling(check2->GetTransform()).matrix();
    Eigen::Vector3d a = Eigen::Vector3d(b1.sizes()[0] * Scale1.row(0)(0), b1.sizes()[1] * Scale1.row(1)(1), b1.sizes()[2] * Scale1.row(2)(2)) / 2;
    Eigen::Vector3d b = Eigen::Vector3d(b2.sizes()[0] * Scale2.row(0)(0), b2.sizes()[1] * Scale2.row(1)(1), b2.sizes()[2] * Scale2.row(2)(2)) / 2;
    Eigen::Vector4d Center1 = Eigen::Vector4d(b1.center()[0], b1.center()[1], b1.center()[2], 1);
    Eigen::Vector4d Center2 = Eigen::Vector4d(b2.center()[0], b2.center()[1], b2.center()[2], 1);
    Eigen::Vector3d D = (check2->GetTransform().cast<double>() * Center2 - check1->GetTransform().cast<double>() * Center1).head(3);

    for (int i = 0; i < 3; i++)
    {
        if (a(i) + (b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(i)(1)) + b(2) * abs(C.row(i)(2))) < abs(A.row(i) * D))
            return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (b(i) + (a(0) * abs(C.row(0)(i)) + a(1) * abs(C.row(1)(i)) + a(2) * abs(C.row(2)(i))) < abs(B.row(i) * D))
            return false;
    }
    int mat[3][4] = { {1, 2, 2, 1 }, { 0, 2, 2, 0 }, { 0, 1, 1, 0 } };
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(1)(i) * A.row(2) * D;
        R = abs(R - C.row(2)(i) * A.row(1) * D);
        if (a(1) * abs(C.row(2)(i)) + a(2) * abs(C.row(1)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(2)(i) * A.row(0) * D;
        R = abs(R - C.row(0)(i) * A.row(2) * D);
        if (a(0) * abs(C.row(2)(i)) + a(2) * abs(C.row(0)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(0)(i) * A.row(1) * D;
        R = abs(R - C.row(1)(i) * A.row(0) * D);
        if (a(0) * abs(C.row(1)(i)) + a(1) * abs(C.row(0)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    return true;
}

void GameScene::checkSnakeCollision()
{
        for (int j = 0; j < prices.size(); j++)
        {
            check1 = snake;
            check2 = prices[j].model;
            if (checkCollision(&sTree, &prices[j].tree)) {
                priceEffect(prices[j].type);
                prices[j].model->Translate(getRandomLocation().cast<float>());
            }
        }
}

void GameScene::movePrices()
{
    for (int i = 0; i < prices.size(); i++) {
        if (tick % 10000==0)
            prices[i].dir = getRandomDir();
        prices[i].model->Translate(prices[i].dir.cast<float>());
    }
}

void GameScene::moveSnake()
{
    moveVertices();
    igl::dqs(V, W, vQ, vT, U);
    snake->SetMeshVertices(U);
    V = U;
    for (int i = 0; i < joints.size(); ++i) {
        Eigen::Vector3d pos_offset = vT[i] - skeleton[i];
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(pos_offset.reverse(), skeleton[i].reverse());
        joints[i]->Translate(pos_offset.reverse().cast<float>());
        joints[i]->Rotate(quat.toRotationMatrix().cast<float>());
    }

    for (int i = 0; i < skeleton.size(); ++i)
        skeleton[i] = vT[i];
}

void GameScene::moveVertices()
{
    vT[0] = skeleton[0];

    for (int i = 0; i < 16; i++) {
        vT[i+1] = skeleton[i+1];
        vT[i] += (vT[i+1] - vT[i]) / 6;
    }

    vT[16] += dir.cast<double>();
}

void GameScene::createW()
{
    V = snake->GetMeshList()[0]->data[0].vertices;
    W.resize(V.rows(), 16 + 1);
    double z, w1, w2, lBound, uBound;
    for (int i = 0; i < V.rows(); i++) {
        z = 10 * V.row(i).z();
        lBound = floor(z);
        uBound = ceil(z);
        w1 = abs(z - uBound);
        w2 = 1 - w1;
        W.row(i) = createWi(w1, lBound + 8, w2, uBound + 8);
    }
}
Eigen::VectorXd GameScene::createWi(double w1, int idx_w1, double w2, int idx_w2)
{
    Eigen::VectorXd Wi;
    Wi.resize(16 + 1);
    for (int i = 0; i < Wi.size(); i++)
    {
        if (i == idx_w1)
            Wi[idx_w1] = w1;
        else if (i == idx_w2)
            Wi[idx_w2] = w2;
        else
         Wi[i] = 0;
    }
    return Wi;
}

Eigen::Vector3d GameScene::getRandomDir()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<int> d(-10, 10);
    return (Eigen::Vector3d(d(rng), d(rng), d(rng)))/100000;
}




GameScene::GameScene(std::string name, Display* display) : SceneWithImGui(std::move(name), display)
{
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
}

void GameScene::SetCamera(int index)
{
    camera = camList[index];
    viewport->camera = camera;
}

void GameScene::Init(float fov, int width, int height, float near, float far)
{
    camList.resize(2);
    camList[0] = Camera::Create("UpperCam", fov, float(width) / float(height), near, far);
    camList[1] = Camera::Create("PovCam", fov, float(width) / float(height), near, far);
    camera = camList[0];
    // create the basic elements of the scene
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    AddChild(pricesRoot = Movable::Create("pricesRoot"));
    auto program = std::make_shared<Program>("shaders/basicShader"); 
    carbon = std::make_shared<Material>("carbon", program); 
    carbon->AddTexture(0, "textures/snake.jpg", 2);
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);

    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
    auto snakeM{ ObjLoader::ModelFromObj("snake","C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/data/snake3.obj", carbon) };
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };
    snake = AutoMorphingModel::Create(*snakeM, morphFunc);
    root->AddChild(snake);
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };
    auto material{ std::make_shared<Material>("material", "shaders/phongShader") };
    trees.resize(16);
    for (int i = 0; i < 16; i++) {
        joints.push_back(Model::Create("joint", cylMesh, material));
        joints[i]->isHidden = true;
        trees[i].init(joints[i]->GetMeshList()[0]->data[0].vertices, joints[i]->GetMeshList()[0]->data[0].faces);
    }
    sTree.init(snake->GetMeshList()[0]->data[0].vertices, snake->GetMeshList()[0]->data[0].faces);
    vT.resize(16 + 1);
    vQ.resize(16 + 1);
    //V = snake->GetMeshList()[0]->data[0].vertices;
    double z = 0, linkL=0.1;
    for (int i = 0; i <= 16; ++i)
    {
        skeleton.push_back(z * Eigen::Vector3d::UnitZ());
        z += linkL;
    }
    createW();
    //pov set
    snake->AddChild(camList[1]);
    camList[1]->RotateByDegree(180, Axis::Y);
    camList[1]->Translate(1, Axis::Z);

    //high set
    camList[0]->Translate(40, Axis::Y);
    camList[0]->RotateByDegree(-90, Axis::X);
    mats.resize(4);
    mats[0] = std::make_shared<Material>("type0", "shaders/BasicShader");
    mats[0]->AddTexture(0, "textures/bricks.jpg", 2);
    mats[1] = std::make_shared<Material>("type1", "shaders/BasicShader");
    mats[1]->AddTexture(0, "C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/textures/Green.jpg", 2);
    mats[2] = std::make_shared<Material>("type2", "shaders/BasicShader");
    mats[2]->AddTexture(0, "C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/textures/Red.jpg", 2);
    mats[3] = std::make_shared<Material>("type3", "shaders/BasicShader");
    mats[3]->AddTexture(0, "C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/textures/golden.jpg", 2);
    initLevel();
}

void GameScene::Update(const Program& p, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(p, proj, view, model);
    gameTick();

}

void GameScene::LoadObjectFromFileDialog()
{
    std::string filename = igl::file_dialog_open();
    if (filename.length() == 0) return;

    auto shape = Model::Create(filename, carbon);
}

void GameScene::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
        case GLFW_KEY_UP:
            dir = { 0,0,-velo };
            snake->Rotate(-snake->GetRotation());
            break;
        case GLFW_KEY_DOWN:
            dir = { 0,0,velo };
            snake->Rotate(-snake->GetRotation());
            snake->RotateByDegree(180, Axis::Y);
            break;
        case GLFW_KEY_LEFT:
            dir = { -velo,0,0 };
            snake->Rotate(-snake->GetRotation());
            snake->RotateByDegree(90,Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            dir = { velo,0,0 };
            snake->Rotate(-snake->GetRotation());
            snake->RotateByDegree(-90, Axis::Y);
            break;
        case GLFW_KEY_I:
            dir = { 0,-velo,0 };
            snake->Rotate(-snake->GetRotation());
            snake->RotateByDegree(90, Axis::X);
            break;
        case GLFW_KEY_O:
            dir = { 0,velo,0 };
            snake->Rotate(-snake->GetRotation());
            snake->RotateByDegree(-90, Axis::X);
            break;
        }
    }

    //SceneWithImGui::KeyCallback(nullptr, x, y, key, scancode, action, mods);
}

void GameScene::ViewportSizeCallback(Viewport* _viewport)
{
    for (auto& cam: camList)
        cam->SetProjection(float(_viewport->width) / float(_viewport->height));

    // note: we don't need to call Scene::ViewportSizeCallback since we are setting the projection of all the cameras
}

void GameScene::AddViewportCallback(Viewport* _viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
}

Eigen::Vector3d GameScene::getRandomLocation() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<int> dZ(-5, 5);
    std::uniform_int_distribution<int> d(-7, 7);
    return Eigen::Vector3d(d(rng), d(rng), dZ(rng));
}

void GameScene::addPrices(int type, int amount)
{
    if (type == 0) {
        for (int i = 0; i < amount; i++) {
            auto priceM = ObjLoader::ModelFromObj("price", "C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/data/cube_old.obj", mats[type]);
            priceM->Translate(getRandomLocation().cast<float>());
            pricesRoot->AddChild(priceM);
            igl::AABB<Eigen::MatrixXd, 3> t;
            t.init(priceM->GetMeshList()[0]->data[0].vertices, priceM->GetMeshList()[0]->data[0].faces);
            prices.push_back({ type,priceM,t,getRandomDir()});
        }
    }
    else {
        for (int i = 0; i < amount; i++) {
            auto priceM = ObjLoader::ModelFromObj("price", "C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/data/sphere.obj", mats[type]);
            priceM->Translate(getRandomLocation().cast<float>());
            pricesRoot->AddChild(priceM);
            igl::AABB<Eigen::MatrixXd, 3> t;
            t.init(priceM->GetMeshList()[0]->data[0].vertices, priceM->GetMeshList()[0]->data[0].faces);
            prices.push_back({ type,priceM,t,getRandomDir() });
        }
    }
}

void GameScene::StartGui() {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 350), ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(300, -1.0), ImVec2(325, -1.0));
    ImGui::Begin("Start Menu - Snake", nullptr, ImGuiWindowFlags_NoMove);
    ImGui::Text("Name: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::InputText("", pName, IM_ARRAYSIZE(pName));
    ImGui::Dummy(ImVec2(0, 10));
    if (state == States::LOSE) {
        if (ImGui::Button("Try Again", ImVec2(-1, 0))) {
            if (strlen(pName) == 0)
                nameErr = true;
            else
                state = States::PLAY;
        }
    }
    if (state == States::WIN) {
        if (ImGui::Button("Play Again", ImVec2(-1, 0))) {
            if (strlen(pName) == 0)
                nameErr = true;
            else
                state = States::PLAY;
        }
    }
    if (state == States::MENU) {
        if (ImGui::Button("Start Game", ImVec2(-1, 0))) {
            if (strlen(pName) == 0)
                nameErr = true;
            else
                state = States::PLAY;
        }
    }
    if(nameErr)
        ImGui::Text("Please Enter Your Name!");
    ImGui::Dummy(ImVec2(0, 5));
    if (ImGui::Button("Tutorial", ImVec2(-1, 0)))
        show = !show;
    static int e = 0;
    ImGui::RadioButton("Upper View", &e, 0); ImGui::SameLine();
    ImGui::RadioButton("POV", &e, 1);
    SetCamera(e);
    if (show) {
        ImGui::BulletText("Use the arrows, 'i' and 'o' to move");
        ImGui::BulletText("Red balls gives health");
        ImGui::BulletText("Green balls gives speed");
        ImGui::BulletText("Bricks deacreases health(avoid)");
        ImGui::BulletText("Golden balls to finish the level/game");
    }
    ImGui::Dummy(ImVec2(0,10));
    std::string highs = "Hall_Of_Fame:";
    for (int i = 0; i < highs.size(); i++) {
        if (i % 3 == 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(252, 248, 5, 255));
            ImGui::Text(highs.substr(i,1).c_str());
            ImGui::SameLine(0, 0);
            ImGui::PopStyleColor();
        }
        else if (i % 3 == 1) {
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
            ImGui::Text(highs.substr(i, 1).c_str());
            ImGui::SameLine(0, 0);
            ImGui::PopStyleColor();
        }
        else {
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
            ImGui::Text(highs.substr(i, 1).c_str());
            ImGui::SameLine(0, 0);
            ImGui::PopStyleColor();
        }
    }
    ImGui::NewLine();
    ImGui::Dummy(ImVec2(0, 5));
    std::vector <save> scores = saves->getScores();
    if (scores.size() == 0) {
        ImGui::BulletText("No Saved Scores!");
    }
    else {
        for (int i = scores.size() - 1; i >= 0; i--) {
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(235, 186, 52, 255));
            std::string text = "Name: " + scores[i].name + " | " + "Score: " + std::to_string(scores[i].score) + "\n";
            ImGui::BulletText(text.c_str());
            ImGui::PopStyleColor();
        }
    }
    ImGui::End();
}

void GameScene::PlayGui() {

    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(300, -1.0f), ImVec2(325, -1.0f));
    ImGui::Begin("Play Menu - Snake", nullptr, ImGuiWindowFlags_NoMove);
    ImGui::Text("Player Name: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(pName);
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(5, 252, 248, 255));
    ImGui::Text("Level: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(std::to_string(level).c_str());
    ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 255, 255));
    ImGui::Text("Score: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(std::to_string(score).c_str());
    ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(252, 248, 5, 255));
    ImGui::Text("Golden caught: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(std::to_string(progress).c_str());
    ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("Health: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(std::to_string(health).c_str());
    ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
    ImGui::Text("Speed: ");
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text(std::to_string(velo).c_str());
    ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0, 10));
    static int e = 0;
    ImGui::RadioButton("Upper View", &e, 0); ImGui::SameLine();
    ImGui::RadioButton("POV", &e, 1);
    SetCamera(e);
    if (state == States::PLAY) {
        if (ImGui::Button("Pause", ImVec2(-1, 0)))
        {
            state = States::PAUSE;
        }
    }
    if (state == States::PAUSE) {
        if (ImGui::Button("Resume", ImVec2(-1, 0)))
        {
            state = States::PLAY;
        }
    }

    ImGui::Dummy(ImVec2(0, 5));

    if (ImGui::Button("Restart level", ImVec2(-1, 0)))
    {
        restartLevel();
        state=States::PLAY;
    }
    ImGui::Dummy(ImVec2(0, 5));
    if (ImGui::Button("Back to menu", ImVec2(-1, 0)))
    {
        state = States::MENU;
    }
    ImGui::End();
}



