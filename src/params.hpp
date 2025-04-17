/*
*
*
*/

template <typename T>
std::map<std::string, T> Params::load(std::string path)
{
    YAML::Node config = YAML::LoadFile(path);
    
    for (const auto& pair : config)
    {
        std::string type_key = pair.first.as<std::string>();
    }  

    std::string type_str = typeid(T).name();
    if (type_str != type_key) 
    {
        throw std::runtime_error("The type you passed is not equal to that in the yaml file.");
    }

    std::map<std::string, T> params;
    for (const auto& pair : params[type_key])
    {
        params[pair.first] = pair.second;
    }

    return params;
}
