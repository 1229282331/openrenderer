#include "config.hpp"

namespace openrenderer{

std::unordered_map<std::string, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders_map = {
    {"point_VertexShader", &point_VertexShader},
    {"nmap_VertexShader", &nmap_VertexShader}
};

std::unordered_map<std::string, std::function<Eigen::Vector3f(const Point&)>> fragmentShaders_map = {
    {"point_FragmentShader", &point_FragmentShader},
    {"triangle_FragmentShader", &triangle_FragmentShader},
    {"normal_FragmentShader", &normal_FragmentShader},
    {"depth_FragmentShader", &depth_FragmentShader},
    {"shadowmap_FragmentShader", &shadowmap_FragmentShader},
    {"gbuffer_FragmentShader", &gbuffer_FragmentShader},
    {"mipmap_FragmentShader", &mipmap_FragmentShader},
    {"ssao_FragmentShader", &ssao_FragmentShader},
    {"ssdo_FragmentShader", &ssdo_FragmentShader},
    {"texture_FragmentShader", &texture_FragmentShader},
    {"albedo_FragmentShader", &albedo_FragmentShader},
    {"phong_FragmentShader", &phong_FragmentShader},
    {"mirror_FragmentShader", &mirror_FragmentShader},
    {"ssr_FragmentShader", &ssr_FragmentShader}, 
    {"normalMapping_FragmentShader", &normalMapping_FragmentShader},
    {"bumpMapping_FragmentShader", &bumpMapping_FragmentShader},
    {"displaceMapping_FragmentShader", &displaceMapping_FragmentShader}
};

Config::Config(const char* file)
{
    Json::Reader reader;
	Json::Value root;
 
	std::ifstream in(file, std::ios::binary);
	if (!in.is_open())
	{
		printf("[error] Can't open the config file!\n");
		state = ConfigState::ERROR;
	}

    if (reader.parse(in, root))
	{
        //1.parse the obj config
		for(unsigned int i=0; i<root["objects"].size(); i++)
		{
            // 1.add obj name
			std::string name = root["objects"][i]["name"].asString();
            obj_names.insert(std::pair<std::string, unsigned int>(name, i));
            // 2.add .obj filepath
            obj_paths.push_back(root["objects"][i]["obj_path"].asString());
            // 3.add obj color
            enableColors.push_back(root["objects"][i]["color"]["enable"].asBool());
			Eigen::Vector3f rgbColor = Eigen::Vector3f::Zero();
			for(unsigned int j=0; j<root["objects"][i]["color"]["rgb"].size(); j++)
				rgbColor[j] = root["objects"][i]["color"]["rgb"][j].asFloat();
            obj_colors.push_back(rgbColor);
            // 4.add obj ptexture
            std::string colorTexture_file = root["objects"][i]["textures"]["colorTexture_path"].asString();
            Texture* colorTexture = colorTexture_file.empty()||enableColors[i]? nullptr : new Texture(colorTexture_file);
			obj_colorTextures.push_back(colorTexture);
            std::string normalTexture_file = root["objects"][i]["textures"]["normalTexture_path"].asString();
            Texture* normalTexture = normalTexture_file.empty()? nullptr : new Texture(normalTexture_file);
            obj_normalTextures.push_back(normalTexture);
            // 5.add obj shaders
            std::string vertexShader_name = root["objects"][i]["shaders"]["vertex_shader"].asString();
            auto vertexShader = vertexShaders_map.find(vertexShader_name)==vertexShaders_map.end()?vertexShaders_map["nmap_VertexShader"]:vertexShaders_map.find(vertexShader_name)->second;
            std::string fragmentShader_name = root["objects"][i]["shaders"]["fragment_shader"].asString();
            auto fragmentShader = fragmentShaders_map.find(fragmentShader_name)==fragmentShaders_map.end()?fragmentShaders_map["point_FragmentShader"]:fragmentShaders_map.find(fragmentShader_name)->second;
            obj_vertexShaders.push_back(vertexShader);
            obj_fragmentShaders.push_back(fragmentShader);
            // 6.add obj model matrix
            Eigen::Vector3f scaleVec = Eigen::Vector3f::Ones();
            for(unsigned int j=0; j<root["objects"][i]["transform"]["scale"].size(); j++)
                scaleVec[j] = root["objects"][i]["transform"]["scale"][j].asFloat();
			Eigen::Vector3f translateVec = Eigen::Vector3f::Zero();
            for(unsigned int j=0; j<root["objects"][i]["transform"]["translate"].size(); j++)
                translateVec[j] = root["objects"][i]["transform"]["translate"][j].asFloat();
			float angle = root["objects"][i]["transform"]["rotate"]["angle"].asFloat();
			Eigen::Vector3f axis = { 0.f, 1.f, 0.f };
            for(unsigned int j=0; j<root["objects"][i]["transform"]["rotate"]["axis"].size(); j++)
                axis[j] = root["objects"][i]["transform"]["rotate"]["axis"][j].asFloat();
            Eigen::Matrix4f modelMatrix = translate(translateVec) * rotate(angle/180.f*float(MY_PI), axis) * scale(scaleVec.x(), scaleVec.y(), scaleVec.z());
            obj_modelMatrixs.push_back(modelMatrix);			
		}
		//2.parse the light config
        for(unsigned int i=0; i<root["lights"].size(); i++)
        {
            LightType type = LightType(root["lights"][i]["type"].asInt());
            Eigen::Vector3f position = {10.f, 10.f, 10.f};
            for(unsigned int j=0; j<root["lights"][i]["position"].size(); j++)
                position[j] = root["lights"][i]["position"][j].asFloat();
            Eigen::Vector3f intensity = {100.f, 100.f, 100.f};
            for(unsigned int j=0; j<root["lights"][i]["intensity"].size(); j++)
                intensity[j] = root["lights"][i]["intensity"][j].asFloat();
            Eigen::Vector3f direction = {0.f, 0.f, 0.f};
            for(unsigned int j=0; j<root["lights"][i]["direction"].size(); j++)
                direction[j] = root["lights"][i]["direction"][j].asFloat();
            bool enable_shadowmap = root["lights"][i]["enable_shadowmap"].asBool();
            Light light = { type, position, intensity, direction.normalized(), enable_shadowmap, Eigen::Matrix4f::Identity(), nullptr };
            lights.push_back(light);
        }
        //3.parse the scene config
        width = root["width"].asInt();
        height = root["height"].asInt();
        cameraPos = { 0.f, 0.f, -5.f };
        for(unsigned int j=0; j<root["cameraPos"].size(); j++)
            cameraPos[j] = root["cameraPos"][j].asFloat();
        lookAt = {0.f, 0.f, 0.f};
        for(unsigned int j=0; j<root["lookAt"].size(); j++)
            lookAt[j] = root["lookAt"][j].asFloat();     
        upDir = {0.f, 1.f, 0.f};
        for(unsigned int j=0; j<root["upDir"].size(); j++)
            upDir[j] = root["upDir"][j].asFloat();  
        fovy = root["fovy"].asFloat();
        zNear = root["zNear"].asFloat();
        zFar = root["zFar"].asFloat();
        enableDefferedRender = root["enableDefferedRender"].asBool();

        printf("[success] Parse config-file complete.\n");
        state = ConfigState::OK;
	}
	else
	{
		printf("[error] Parse config-file error!\n");
        state = ConfigState::ERROR;
	}
 
	in.close();
}

Config::~Config()
{
    for(unsigned int i=0; i<obj_colorTextures.size(); i++)
    {
        if(obj_colorTextures[i])
            delete obj_colorTextures[i];
        obj_colorTextures[i] = nullptr;
    }
    for(unsigned int i=0; i<obj_normalTextures.size(); i++)
    {
        if(obj_normalTextures[i])
            delete obj_normalTextures[i];
        obj_normalTextures[i] = nullptr;
    }
}

}