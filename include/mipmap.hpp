#pragma once
#include "buffer.hpp"


namespace openrenderer{

template <typename T>
T mean(T lt, T lb, T rt, T rb)
{
    return T((lt+lb+rt+rb) / T(4));
}
template <typename T>
T min(T lt, T lb, T rt, T rb)
{
    return std::min(std::min(lt, lb), std::min(rt, rb));
}
template <typename T>
T max(T lt, T lb, T rt, T rb)
{
    return std::max(std::max(lt, lb), std::max(rt, rb));
}

template <typename T, int levels>
class MipMap{
public:
    MipMap(const Buffer<T>& sourceMap) : m_levels(levels), m_pbyte(sourceMap.pbyte), m_format(sourceMap.format)
    {
        Buffer<T>* mipmap = new Buffer<T>(sourceMap);
        m_maps.push_back(mipmap);
        for(int i=0; i<levels; i++)
        {
            mipmap = new Buffer<T>(m_format, m_maps[m_maps.size()-1]->width/2, m_maps[m_maps.size()-1]->height/2);
            #pragma omp parallel for num_threads(16)
            for(int i=0;i<=m_maps[m_maps.size()-1]->height-2; i+=2)
                for(int j=0; j<=m_maps[m_maps.size()-1]->width-2; j+=2)
                {
                    for(int k=0; k<mipmap->pbyte; k++)
                    {
                        T value = max(m_maps[m_maps.size()-1]->get(i, j, ColorBit(k)), m_maps[m_maps.size()-1]->get(i+1, j, ColorBit(k)), m_maps[m_maps.size()-1]->get(i, j+1, ColorBit(k)), m_maps[m_maps.size()-1]->get(i+1, j+1, ColorBit(k)));
                        mipmap->set(i/2, j/2, ColorBit(k), value);
                    }
                }
            m_maps.push_back(mipmap);
        }  
    }
    ~MipMap()
    {
        for(int i=0; i<m_levels+1; i++)
            delete m_maps[i];
        m_maps.clear();
    }
    std::vector<Buffer<T>*> maps() { return m_maps; }

private:
    int m_levels;
    int m_pbyte;
    PixelFormat m_format;
    std::vector<Buffer<T>*> m_maps;
};


}