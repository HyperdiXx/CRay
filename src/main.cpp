
#include <iostream>
#include <vector>
#include <thread>
#include <random> 
#include <algorithm>
#include <filesystem>

#define M_PI 3.14159265358979323846

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define bytesPerPixel 3

inline float clamp(float t, float min, float max) {
    if (t < min) return min;
    if (t > max) return max;
    return t;
}

inline float frand(int *seed)
{
    union
    {
        float fres;
        unsigned int ires;
    };

    seed[0] *= 16807;
    ires = ((((unsigned int)seed[0])>>9 ) | 0x3f800000);
    return fres - 1.0f;
}

inline float randf()
{
    //static std::random_device rd;
    //static std::mt19937 gen(rd());
    //static std::uniform_real_distribution<float> uniDist { 0.0f, 1.0f };
    //return uniDist(gen);
    static int seed = 15677;
    return frand(&seed);    
}

struct Vec2
{
    float x, y;
};

struct Vec3
{
    float x, y, z;

    Vec3() { }
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { }

    inline void normalize()
    {
        float invlength = 1.0f / sqrt(x * x + y * y + z * z);
        
        if (invlength > 0.0f)
        {
            this->x *= invlength;
            this->y *= invlength;
            this->z *= invlength;   
        }
    }

    inline Vec3 negated() const
    {
        return { -x, -y, -z };
    }

    inline static Vec3 normalize(const Vec3 &a)
    {
        Vec3 result = {};
        float invlength = 1.0f / sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        //float length = sqrt(x * x + y * y + z * z);
        
        if (invlength > 0.0f)
        {
            result.x = a.x * invlength;
            result.y = a.y * invlength;
            result.z = a.z * invlength;   
        }

        return result;
    }

    inline float lengthSqrt()
    {
        return x * x + y * y + z * z;
    }

    inline static float dot(const Vec3 &a, const Vec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline static Vec3 cross(const Vec3 &a, const Vec3 &b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    inline static Vec3 reflect(const Vec3 &dir, const Vec3 &normal)
    {
        return dir - 2.0f * Vec3::dot(dir, normal) * normal;
    }

    friend Vec3 operator+(const Vec3 &a, const Vec3 &b)
    {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    friend Vec3 operator-(const Vec3 &a, const Vec3 &b)
    {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    friend Vec3 operator*(float a, const Vec3 &b)
    {
        return { b.x * a, b.y * a, b.z * a };
    }

    friend Vec3 operator*(const Vec3 &b, float a)
    {
        return { b.x * a, b.y * a, b.z * a };
    }

    friend Vec3 operator*(const Vec3 &a, const Vec3 &b)
    {
        return { a.x * b.x, a.y * b.y, a.z * b.z };
    }

    friend Vec3 operator+(const Vec3 &a, float val)
    {   
        return { a.x + val, a.y + val, a.z + val };
    }    

    friend Vec3 operator/(const Vec3 &a, float b)
    {   
        return { a.x / b, a.y / b, a.z / b };
    }
  
    float operator[](int index) const
    {
        assert(index <= 0 || index > 2);
        if (index == 0)
        {
            return x;
        }
        else if (index == 1)
        {
            return y;
        }
        else
        {
            return z;
        }
    }

    float& operator[](int index)
    {
        assert(index < 0 || index > 2);
        if (index == 0)
        {
            return x;
        }
        else if (index == 1)
        {
            return y;
        }
        else
        {
            return z;
        }
    }

    Vec3 &operator+=(const Vec3 &a)
    {
        this->x += a.x;
        this->y += a.y;
        this->z += a.z;

        return *this;
    }

    Vec3 &operator*=(float a)
    {
        this->x *= a;
        this->y *= a;
        this->z *= a;
        
        return *this;
    }

    Vec3 &operator*=(const Vec3 &a)
    {
        this->x *= a.x;
        this->y *= a.y;
        this->z *= a.z;
        
        return *this;
    }    

    Vec3 &operator/=(float a)
    {
        this->x /= a;
        this->y /= a;
        this->z /= a;
        
        return *this;
    }

    Vec3 &operator/=(const Vec3 &a)
    {
        this->x /= a.x;
        this->y /= a.y;
        this->z /= a.z;
        
        return *this;
    }
};

struct AABB
{
    Vec3 m_min, m_max;
    Vec3 m_slabs[3];

    AABB() { }
    AABB(const Vec3 &min, const Vec3 &max) : m_min(min), m_max(max) { }
    AABB(const Vec3 &xslab, const Vec3 &yslab, const Vec3 &zslab) :  m_slabs { xslab, yslab, zslab } { }    

    bool hit(const Vec3 &orig, const Vec3 &dir, float &tmin, float &tmax) const
    {
        for (int i = 0; i < 3; ++i)
        {
            auto invDir = 1.0f / dir[i];
            auto t0 = (m_min[i] - orig[i]) * invDir;
            auto t1 = (m_max[i] - orig[i]) * invDir;
            if (invDir < 0.0f)
            {
                std::swap(t0, t1);
            }   

            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;
            if (tmax <= tmin)
            {
                return false;
            }            
        }

        return true;
    }

    const Vec3& getXSlab() const { return m_slabs[0]; }
    const Vec3& getYSlab() const { return m_slabs[1]; } 
    const Vec3& getZSlab() const { return m_slabs[2]; } 
    const Vec3& getSlab(int ind) const { return m_slabs[ind]; }

    static AABB calculateSurround(const AABB &a, const AABB &b)
    {
        Vec3 small = { fmin(a.m_min.x, b.m_min.x), 
                       fmin(a.m_min.y, b.m_min.y), 
                       fmin(a.m_min.z, b.m_min.z) };

        Vec3 big = { fmax(a.m_max.x, b.m_max.x),
                     fmax(a.m_max.y, b.m_max.y), 
                     fmax(a.m_max.z, b.m_max.z)};

        return { small, big };
    }

};

class Material;

struct HitEntry
{
    Vec3 normal;
    Vec3 p;
    float t;
    const Material *m;
    float u, v;
    float colorStep;

    inline void setFaceNormal(const Vec3 &or, const Vec3 &dir, const Vec3 &norm)
    {
        bool front_face = Vec3::dot(dir, norm) < 0;
        Vec3 neg = norm.negated();
        normal = front_face ? norm : neg;
    }

    static void getSphereOnUV(const Vec3& p, float& u, float& v) 
    {
        // p: a given point on the sphere of radius one, centered at the origin.
        // u: returned value [0,1] of angle around the Y axis from X=-1.
        // v: returned value [0,1] of angle from Y=-1 to Y=+1.
        //     <1 0 0> yields <0.50 0.50>       <-1  0  0> yields <0.00 0.50>
        //     <0 1 0> yields <0.50 1.00>       < 0 -1  0> yields <0.50 0.00>
        //     <0 0 1> yields <0.25 0.50>       < 0  0 -1> yields <0.75 0.50>

        auto theta = acos(-p.y);
        auto phi = atan2(-p.z, p.x) + M_PI;

        u = phi / (2 * M_PI);
        v = theta / M_PI;
    }
};

struct Hittable
{
    virtual bool isHit(const Vec3 &orig, const Vec3 &dir, float &tMin, float &tMax, HitEntry &entry) const = 0;
    virtual bool calculateBox(AABB &out) const = 0;
};

class BBox : public Hittable
{
public:
    AABB m_box;
    Vec3 m_pos;
    const Material* mat;

public:

    BBox() {}
    BBox(const Vec3& min, const Vec3& max, const Material *m) : mat(m)
    {
        m_box = AABB(min, max);
    }

    virtual bool isHit(const Vec3& orig, const Vec3& dir, float &tMin, float &tMax, HitEntry& entry) const override
    {       
        bool isHitted = m_box.hit(orig, dir, tMin, tMax);
        
        if (isHitted)
        {
            entry.m = mat;
            entry.p = orig + dir * tMin;
            
            auto x = orig.x + tMin * dir.x;
            auto y = orig.y + tMin * dir.y;
           
            //entry.u = (x - x0) / (x1 - x0);
            //entry.v = (y - y0) / (y1 - y0);
            entry.t = tMin;
            std::cout << "Hitted bbox\n";
        }

        return isHitted;
    }

    virtual bool calculateBox(AABB& out) const override
    {
        out = m_box;
        return true;
    }
};

class BVHNode : Hittable
{
public:
    BVHNode() { }
    BVHNode(const std::shared_ptr<Hittable> &table) { }
    
    /*BVHNode(const HitTable& list)
        : BVHNode(list.getObjects(), 0, list.getObjects().size())
    {

    }*/

    BVHNode(const std::vector<std::shared_ptr<Hittable>> &objects, int start, int end)
    {
        int nhitted = end - start;
        if (nhitted == 1)
        {
            m_left = m_right = objects[0];
        }
        else if (nhitted == 2)
        {
            m_left = objects[0];
            m_right = objects[1];
        }
        else 
        {   
            int cmpAxis = 3.0f * randf();
            auto cmpFunc = [cmpAxis](const std::shared_ptr<Hittable> &a, const std::shared_ptr<Hittable> &b)
            {
                AABB boxLeft = {};
                AABB boxRight = {};

                a->calculateBox(boxLeft);
                b->calculateBox(boxRight);
        
                return boxLeft.getSlab(cmpAxis).x < boxRight.getSlab(cmpAxis).x;
            };

            //std::sort(objects.data() + start, objects.data() + end, cmpFunc);
            int halfRange = start + nhitted * 0.5f;

            //m_left = std::make_shared<BVHNode>(objects, start, halfRange);
            //m_right = std::make_shared<BVHNode>(objects, halfRange, end);
        }

        AABB fB = {};
        AABB sB = {};

        m_left->calculateBox(fB);
        m_right->calculateBox(sB);

        m_box = AABB::calculateSurround(fB, sB);
    }

    virtual bool isHit(const Vec3 &orig, const Vec3 &dir, float &tMin, float &tMax, HitEntry &entry) const override
    {
        if (!m_box.hit(orig, dir, tMin, tMax))
        {
            return false;
        }

        bool isHitLeft = m_left->isHit(orig, dir, tMin, tMax, entry);
        bool isHitRight = m_right->isHit(orig, dir, tMin, isHitLeft ? entry.t : tMax, entry);
        
        return isHitLeft || isHitRight;
    }
    
    virtual bool calculateBox(AABB &out) const override
    {
        out = m_box;
        return true;
    }
    
private:

    std::shared_ptr<Hittable> m_left;
    std::shared_ptr<Hittable> m_right;
    
    AABB m_box;
    
};

class Texture2D
{
public:

    Texture2D() : data(nullptr), w(0), h(0) { }
    Texture2D(const char* tex)
    {
        loadFromFile(tex);
    }

    ~Texture2D() { free(data); }

    void setSize(float width, float height)
    {
        w = width; h = height;

        if (!data)
        {
            data = (unsigned char*)malloc(w * h * bytesPerPixel);
        }
        else
        {
            data = (unsigned char*)realloc(data, w * h * bytesPerPixel);
        }
    }
    
    void noise();
    
    void loadFromFile(const char *path)
    {
        auto components = bytesPerPixel;

        stbi_set_flip_vertically_on_load(true);
        data = stbi_load(path, &w, &h, &components, components);

        if (!data) 
        {
            std::cerr << "ERROR: Could not load texture image file '" << path << "'.\n";
            w = 0;
            h = 0;
        }

        bytesPerRow = bytesPerPixel * w;
    }

    void white()
    {
        Vec3 white = { 1.0f, 1.0f, 1.0f };
        color(white);
    }
    
    void color(const Vec3& color)
    {
        for (uint32_t y = 0; y < h; ++y)
        {
            for (uint32_t x = 0; x < w; ++x)
            {
                auto index = bytesPerPixel * (y * w + x); 
                
                data[index + 0] = 255.99f * color.x;
                data[index + 1] = 255.99f * color.y;
                data[index + 2] = 255.99f * color.z;                
            }
        }
    }
    
    Vec3 getColor(float u, float v)
    {
        if (!data)
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        // Clamp input texture coordinates to [0,1] x [1,0]
        u = clamp(u, 0.0f, 1.0f);
        v = 1.0f - clamp(v, 0.0f, 1.0f);

        auto i = static_cast<int>(u * w);
        auto j = static_cast<int>(v * h);
        
        if (i >= w)  i = w - 1;
        if (j >= h)  j = h - 1;

        constexpr static auto scale = 1.0f / 255.0f;

        auto pixel = data + j * bytesPerRow + i * bytesPerPixel;

        return { scale * pixel[0], scale * pixel[1], scale * pixel[2] };
    }

    Vec3 getPixel(uint32_t index)
    {
        if (!data)
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        assert(index >= 0);
        constexpr static auto scale = 1.0f / 255.0f;
        
        float pxl1 = data[index];
        float pxl2 = data[index + 1];
        float pxl3 = data[index + 2];

        return { scale * pxl1, scale * pxl2, scale * pxl3 };
    }

    inline int getWidth() const { return w; }
    inline int getHeight() const { return h; } 

private:
    unsigned char *data;
    int w, h;
    int bytesPerRow;
};

Vec3 getPointOnUnitSphere()
{
    Vec3 res;
    do
    {
        res = 2.0f * Vec3(randf(), randf(), randf()) - Vec3(1.0f, 1.0f, 1.0f);
    } while(res.lengthSqrt() > 1.0f);

    return res;
}


Vec3 randomInHemisphere(const Vec3& normal) {
    Vec3 pointInSphere = getPointOnUnitSphere();
    float dot = Vec3::dot(pointInSphere, normal);

    if (dot > 0.0)
    {
        // In the same hemisphere as the normal
        return pointInSphere;
    } 
    else
    {
        return { -pointInSphere.x, -pointInSphere.y, -pointInSphere.z };
    }
        
}

struct HitEntry;

class Ray
{
public:
    Ray() { }
    Ray(const Vec3 &o, const Vec3 &dir) : origin(o), direction(dir)
    {
        direction.normalize();
    }

    inline Vec3 getHit(float dist) { return origin + direction * dist; }
    const Vec3 &getOrigin() const { return origin; }
    const Vec3 &getDir() const { return direction; }

private:
    Vec3 origin;
    Vec3 direction;  
};

class Material
{
public:
    virtual ~Material() { }
    virtual bool scatter(const Vec3 &rO, const Vec3 &rD, Vec3 &att, HitEntry &entry, Ray &scattered) const = 0;
    virtual Vec3 emitted(double u, double v) const = 0;
};

class DiffuseMaterial : public Material
{
public:
    DiffuseMaterial(const Vec3 &al) : albedo(al) { }
    virtual ~DiffuseMaterial()
    {

    }

    virtual bool scatter(const Vec3 &rO, const Vec3 &rD, Vec3 &att, HitEntry &entry, Ray &scattered) const override
    {
        const Vec3 targetRes = entry.p + randomInHemisphere(entry.normal);  
        att = albedo;
        scattered = { entry.p, targetRes - entry.p };

        return true;
    }

    virtual Vec3 emitted(double u, double v) const override
    {
        return { 0.0f, 0.0f, 0.0f };
    }

private:
    Vec3 albedo;

};

class MetalMaterial : public Material
{
public:
    MetalMaterial(const Vec3 &att, float noise) : attn(att), noiseness(noise) { }
    virtual ~MetalMaterial()
    {

    }

    virtual bool scatter(const Vec3 &rO, const Vec3 &rD, Vec3 &att, HitEntry &entry, Ray &scattered) const override
    {
        const Vec3 reflectedDir = Vec3::reflect(rD, entry.normal) + randomInHemisphere(entry.normal) * noiseness;  
        att = attn;
        scattered = { entry.p, reflectedDir };

        //return Vec3::dot(scattered.getDir(), entry.normal) > 0.0f;
        return true;
    }

    virtual Vec3 emitted(double u, double v) const override
    {
        return { 0.0f, 0.0f, 0.0f };
    }


private:
    Vec3 attn;
    float noiseness;
};

class IsotropicMaterial : public Material
{
public:
    IsotropicMaterial(const Texture2D& color)
    {
        m_albedo = std::make_shared<Texture2D>(color);
    }
    
    virtual ~IsotropicMaterial()
    {

    }

    virtual bool scatter(const Vec3& rO, const Vec3& rD, Vec3& att, HitEntry& entry, Ray& scattered) const override
    {
        att = m_albedo->getColor(entry.u, entry.v);
        scattered = { entry.p, randomInHemisphere(entry.normal) };

        return true;
    }

    virtual Vec3 emitted(double u, double v) const override
    {
        return { 0.0f, 0.0f, 0.0f };
    }


private:
    std::shared_ptr<Texture2D> m_albedo;
};


struct Sphere : Hittable
{
    float radius;
    Vec3 center;
    const Material *mat;

    Sphere(const Vec3 &c, float r, const Material *m) : center(c), radius(r), mat(m) { }
    
    bool intersects(const Vec3 &orig, const Vec3 &dir, float &t)
    {
        Vec3 dist = center - orig;
        float dirRay = Vec3::dot(dist, dir);
        float distSqr = Vec3::dot(dist, dist) - dirRay * dirRay;

        if (distSqr > radius * radius)
        {
            return false;
        }

        float thc = sqrtf(radius * radius - distSqr);
        t = dirRay - thc;

        float t_min = dirRay + thc;

        if (t < 0) 
        {
            t = t_min;
        }

        if (t < 0)
        {
            return false;
        } 

        return true;
    }

    virtual bool isHit(const Vec3 &orig, const Vec3 &dir, float &tMin, float &tMax, HitEntry &entry) const override
    {
        Vec3 oc = orig - center;
        float a = Vec3::dot(dir, dir);
        float b = 2.0f * Vec3::dot(oc, dir);
        float c = Vec3::dot(oc, oc) - radius * radius;
        float d = b * b - 4.0f * a * c;
        bool result = d > 0.0f;
        
        if (result)
        {
            const float sol1 = (-b - sqrtf(d)) / (2.0f * a); 
            const float sol2 = (-b + sqrtf(d)) / (2.0f * a);
            
            if (sol1 > tMin && sol1 < tMax)
            {
                entry.t = sol1;
            } 
            else if (sol2 > tMin && sol2 < tMax)
            {
                entry.t = sol2;
            }
            else 
            {
                return false;
            }
            
            Vec3 point = orig + dir * entry.t; 
            Vec3 res = Vec3::normalize(point - center);
            entry.p = point;
            entry.normal = res;
            entry.m = mat; 
            Vec3 outNormal = (entry.p - center) / radius;
            entry.setFaceNormal(orig, dir, outNormal);
            HitEntry::getSphereOnUV(outNormal, entry.u, entry.v);
        }

        return result;
    }

    virtual bool calculateBox(AABB &out) const override
    {   
        out = {
            center - Vec3(radius, radius, radius),
            center + Vec3(radius, radius, radius) 
        };

        return true;
    }
};

class DiffLight : public Material 
{
public:
    DiffLight(const std::shared_ptr<Texture2D> &a) : m_emitTexture(a) {}
    DiffLight(const Vec3 &color) 
    {
        m_emitTexture = std::make_shared<Texture2D>();
        m_emitTexture->setSize(1.0f, 1.0f);
        m_emitTexture->color(color);
    }

    virtual bool scatter(const Vec3& rO, const Vec3& rD, Vec3& att, HitEntry& entry, Ray& scattered) const override 
    {
        const Vec3 reflectedDir = Vec3::reflect(rD, entry.normal) + randomInHemisphere(entry.normal);
        scattered = { entry.p, reflectedDir }; 
        float dC = fmax(0.0f, Vec3::dot(entry.normal, reflectedDir - entry.p));
        entry.colorStep = dC;

        return false;
    }

    virtual Vec3 emitted(double u, double v) const override 
    {
        return m_emitTexture->getColor(u, v);
    }

public:
    std::shared_ptr<Texture2D> m_emitTexture;
};


Vec3 castRay(const Vec3 &orig, const Vec3 &dir, Sphere &sphere) {
    
    constexpr float sphere_dist = std::numeric_limits<float>::max();
    
    Vec3 nDir = Vec3(dir.x - orig.x, dir.y - orig.y, dir.z - orig.z);
    
    //if (!sphere.hit(orig, nDir, sphere_dist))
    //{
    //    return Vec3(0.2f, 0.4f, 0.7f); 
   // }
    
    std::cout << "Hitting sphere!\n";
        
    return Vec3(0.5f, 0.4f, 0.3f);
}

void printArray(int ar[], int n)
{
    for (int i = 0; i < n; ++i)
    {
        std::cout << ar[i] << " "; 
    }

    std::cout << std::endl;
}

void inserSort(int a[], int n)
{
    for (int i = 1; i < n; ++i)
    {
        int key = a[i];
        int j = i - 1;
        
        std::cout << "Key: " << key << "\n";

        while (j >= 0 && a[j] > key)
        {
            std::cout << "a j: " << a[j] << "\n";            
            std::cout << "a j + 1: " << a[j + 1] << "\n";

            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;

        std::cout << "Print new array: "; 
        printArray(a, 6);        
    }
}

struct FileType
{
    char *data = 0;
    char *fileName;
    bool isLoaded = false;
};

struct Str
{
    static inline char* eatSlashes(char *buf)
    {
        bool isSlashesFormatted = false;

        while (true)
        {
            buf++;
            if (buf[0] == '/' && buf[1] == '/')
            {
                isSlashesFormatted = true;
                buf += 2;
            }

            if (isDigit(buf[0]) && isSlashesFormatted)
            {
                break;
            }

            if (isNewLine(buf[0]))
            {
                break;
            }
        }
        
        return buf;
    }

    static bool isNewLine(char c)
    {
        return c == '\n' || c == '\r' || c == '\t';
    }

    static inline char* eatWhitespaces(char* buf)
    {
        while(true) 
        {
            if (buf[0] == ' ')
            {
                buf++;
            }
            else if (buf[0] == '\\')
            {
                buf += 3;
            }
            else 
            {
                break;
            }
		}

        return buf;
    }

    static bool isLetterUpperCase(char c)
    {
        return c >= 'A' && c <= 'Z';
    }

    static bool isLetterLowerCase(char c)
    {
        return c >= 'a' && c <= 'z';
    }

    static bool isLetter(char c)
    {
        return isLetterLowerCase(c) || isLetterUpperCase(c);
    }

    static bool isDigit(char c)
    {
        return (c >= (int)'0') && (c <= (int)'9');
    }

    static bool isDigitOrLetter(char c)
    {
        return isDigit(c) || isLetter(c);
    }

    static inline int strWidth(char **buffer)
    {
        char* buf = *buffer;
	    int i = 0;
	    while(buf[i] != '\r' && buf[i] != '\n' && buf[i] != '\0') i++;
        
        //int newLine = strFind(buf, '\n') - 1;
		//if(newLine < 0) newLine = strLen(buf);

		//if(buf[newLine-1] == '\r') newLine--;
		//return newLine;

        return i - 1;
    };

    static void createStr(char *buf, int size, char *outStr)
    {
        memcpy(outStr, buf, size);        
    }

    static char *allocateStr(char *buf, int size)
    {
        assert(size > 0);
        char *newStr = (char*)malloc(size);
        memcpy(newStr, buf, size);
        return newStr;
    }

    static inline int strFind(char *str, char dst)
    {
        int index = 0;
	    int pos = -1;
	    while(str[index] != '\0') 
        {
		    if(str[index] == dst) 
            {
			    pos = index + 1;
			    break;
		    }
		    index++;
	    }

	    return pos;
    }
};

struct ScopeString
{
    ScopeString(char *src, int size)
    {
        this->data = Str::allocateStr(src, size);
    }

    ~ScopeString()
    {
        if (data)
        {
            free(data);
        }
    }

    char *data;
};

struct FileIO
{
    static FileType readFileToBufferZT(const char *filename)
    {
        FileType result = {};
        FILE* file = fopen(filename, "rb");
	    if(file == 0)
        {
            result.isLoaded = false;
            //strcpy(result.fileName, filename);            
            return result;
        };

	    fseek(file, 0, SEEK_END);
	    int size = ftell(file);
	    fseek(file, 0, SEEK_SET);       
    
        result.data = (char*)malloc(size + 1);
        fread(result.data, size, 1, file);
        result.data[size] = '\n';

        fclose(file);

        result.isLoaded = true;

        return result;
    }

};

struct Vertex
{
    Vec3 position; 
    Vec2 uv;

    Vertex() { }

    Vertex(const Vec3 &vec)
    {
        this->position = vec;
    }

    Vertex(const Vertex &v)
    {
        this->position = v.position;
    }

};

struct Triangle
{
    Vec3 a, b, c;
    

    inline bool isHit(const Vec3 &orig, const Vec3 &dir, HitEntry &out) const
    {
        const Vec3 edge1 = b - a;
		const Vec3 edge2 = c - a;

        // begin calculating determinant - also used to calculate U parameter
        Vec3 crossPro = Vec3::cross(dir, edge2);

		// if determinant is near zero, ray lies in plane of triangle
		float det = Vec3::dot(edge1, crossPro);

		Vec3 perp = {};
        Vec2 baryPosition = {};

		if(det > std::numeric_limits<float>::epsilon())
		{
			// calculate distance from vert0 to ray origin
			Vec3 dist = orig - a;

			// calculate U parameter and test bounds
			baryPosition.x = Vec3::dot(dist, crossPro);
			if(baryPosition.x < static_cast<float>(0) || baryPosition.x > det)
			{
                return false;
            }

			// prepare to test V parameter
			perp = Vec3::cross(dist, edge1);

			// calculate V parameter and test bounds
			baryPosition.y = Vec3::dot(dir, perp);
			if((baryPosition.y < static_cast<float>(0)) || ((baryPosition.x + baryPosition.y) > det))
			{
                return false;
            }
		}
		else if(det < -std::numeric_limits<float>::epsilon())
		{
			// calculate distance from vert0 to ray origin
			Vec3 dist = orig - a;

			// calculate U parameter and test bounds
			baryPosition.x = Vec3::dot(dist, crossPro);
			if((baryPosition.x > static_cast<float>(0)) || (baryPosition.x < det))
			{
                return false;
            }

			// prepare to test V parameter
			perp = Vec3::cross(dist, edge1);

			// calculate V parameter and test bounds
			baryPosition.y = Vec3::dot(dir, perp);
			if((baryPosition.y > static_cast<float>(0)) || (baryPosition.x + baryPosition.y < det))
			{
                return false;
            }
        }
		else
		{
            return false; // ray is parallel to the plane of the triangle
        }

		float invDet = 1.0f / det;

		// calculate distance, ray intersects triangle
		float distance = Vec3::dot(edge2, perp) * invDet;
		//baryPosition *= inv_det;

		return true;
    }
};

struct Face
{
    int i1, i2, i3;
};

struct Mesh : Hittable
{
    std::vector<Vertex> m_vertices;
    std::vector<Face> m_faces;
    std::vector<Triangle> m_triangles;

    Vec3 position;
    const Material* mat;

    Mesh() {}
    
    Mesh(const Vec3 &pos, const Material* m) : position(pos), mat(m) { }
    Mesh(const Mesh* msh, const Material* m) : position(msh->position), mat(m)
    {
        m_vertices = msh->m_vertices;
        m_faces = msh->m_faces;
        m_triangles = msh->m_triangles;
    }

    inline void buildTriangles()
    {
        for (uint32_t i = 0; i < m_triangles.size(); ++i)
        {
            Triangle &tri = m_triangles[i];
            
            tri.a += position;
            tri.b += position;
            tri.c += position;
        }
    }
  
    virtual bool isHit(const Vec3 &orig, const Vec3 &dir, float &tMin, float &tMax, HitEntry &entry) const override
    {
        bool isHit = false;
        for (uint32_t i = 0; i < m_triangles.size(); ++i)
        {
            const Triangle &tri = m_triangles[i];
            bool triHit = tri.isHit(orig, dir, entry);
            
            if(triHit)
            {
                isHit |= triHit;
                break;
            }            
        }

        return isHit;
    }

    virtual bool calculateBox(AABB &out) const override
    {
        AABB box; 

        

        return true;
    }
};

class OBJLoader
{
public:
    
    static Mesh* load(const char *name, const Vec3 &pos, const char *folder = 0)
    {
        static std::filesystem::path cwd = std::filesystem::current_path();
        static std::filesystem::path parent = cwd.parent_path();

        const char* path = std::string(parent.string() + name).c_str();

        FileType file = FileIO::readFileToBufferZT(name);
        if (!file.isLoaded)
        {
            return new Mesh();
        }

        char* charBuf = file.data;

        Mesh *result = new Mesh();

        std::vector<Vertex> verts;
        std::vector<uint32_t> indicesPos;
        std::vector<uint32_t> indicesUV;
        std::vector<uint32_t> indicesNorm;
 
        while (charBuf[0] != 0)
        {            
            char c = charBuf[0];
            char c1 = charBuf[1];

            switch (c)
            {
            case '#':
            {
                // Skipping!
                
                int lineW = Str::strWidth(&charBuf);
                charBuf += lineW;                
               
            } break;
            case 'm': 
            {
                // mtlib file:
                charBuf += 7;
                int lineW = Str::strWidth(&charBuf);
                ScopeString matName(charBuf, lineW); 

                // Parse material file
                // ...

                charBuf += lineW;
            } break;
            case 'o': 
            {
				charBuf += 2;
                int lineW = Str::strWidth(&charBuf);
                ScopeString objName(charBuf, lineW); 
                charBuf += lineW;
			} break;
            case 's':
            {
                charBuf += 2;
                int lineW = Str::strWidth(&charBuf);
                //ScopeString sName(charBuf, lineW); 
                charBuf += lineW;
            } break;
            case 'v':
            {
                if (c1 == ' ')
                {
                    charBuf += 2;
                }
                else
                {
                    charBuf += 3;
                }

                Vertex vertexOut = {};

                if(c1 == 't')
                {
                    Vec2 uv = {};

                    uv.x = atof(charBuf);
                    charBuf += Str::strFind(charBuf, ' ');
                    uv.y = atof(charBuf);
                    
                    vertexOut.uv = uv;

                    break;
                }

                Vec3 vert = {};

				vert.x = atof(charBuf);
				charBuf += Str::strFind(charBuf, ' ');
				vert.y = atof(charBuf);
				charBuf += Str::strFind(charBuf, ' ');
				vert.z = atof(charBuf);
                
                vertexOut.position = vert;

                verts.emplace_back(vertexOut);

            } break;
            case 'u':
            {
                // usemtl
                charBuf += 7;
                
                int lineW = Str::strWidth(&charBuf);
                ScopeString matName(charBuf, lineW); 

                // Parse material file
                // ...

                charBuf += lineW;
            } break;
            case 'f':
            {
                charBuf++;
                
                static bool isSlashesFormatted = charBuf[2] == '/' && charBuf[3] == '/';

                while(true)
                {
                    charBuf = Str::eatWhitespaces(charBuf);
                    if (!Str::isDigit(charBuf[0]))
                    {
                        break;
                    }
                    
                    Face fac = {};
                    
                    if (isSlashesFormatted)
                    {
                        fac.i1 = atoi(charBuf);
                        while(!Str::isDigit(charBuf[0])) charBuf++;
                        charBuf = Str::eatWhitespaces(charBuf);

                        //fac.i2 = atoi(charBuf);
                        //while(Str::isDigit(charBuf[0])) charBuf++;
                        //charBuf = Str::eatWhitespaces(charBuf);
                        
                        charBuf += 3;
                        fac.i3 = atoi(charBuf);
                        while(Str::isDigit(charBuf[0])) charBuf++;         

                        indicesPos.emplace_back(fac.i1 - 1);       
                        indicesNorm.emplace_back(fac.i3 - 1);
                    }
                    else
                    {
                        fac.i1 = atoi(charBuf);
                        while(Str::isDigit(charBuf[0])) charBuf++;

                        if (charBuf[0] == '/')
                        {
                            charBuf++;
                            
                            if (charBuf[0] != '/')
                            {
                                fac.i2 = atoi(charBuf);
                                while(Str::isDigit(charBuf[0])) charBuf++;

                            }

                            if (charBuf[0] == '/')
                            {
                                charBuf++;
                                fac.i3 = atoi(charBuf);
                                while(Str::isDigit(charBuf[0])) charBuf++;
                            }
                            
                            indicesPos.emplace_back(fac.i1 - 1);
                            indicesUV.emplace_back(fac.i2 - 1);
                            indicesNorm.emplace_back(fac.i3 - 1);
                        }   
                    }
             
                }
                
            } break;
            default:
                break;
            }

            charBuf++;
        }
        
        result->m_vertices = std::move(verts);        
        result->position = pos;
            
        for (int i = 0; i < indicesPos.size(); i += 3)
        {                        
            Triangle tri = {};
            uint32_t ind1 = indicesPos[i];
            uint32_t ind2 = indicesPos[i + 1];
            uint32_t ind3 = indicesPos[i + 2]; 
            
            tri.a = result->m_vertices[ind1].position + pos;
            tri.b = result->m_vertices[ind2].position + pos;
            tri.c = result->m_vertices[ind3].position + pos;
            
            result->m_triangles.emplace_back(tri);   
        }
                                             
        
        return result;
    }

private:
    
};

class Framebuffer
{
public:
    Framebuffer() { }
    Framebuffer(uint32_t w, uint32_t h) : width(w), height(h), data((unsigned char*)malloc(w * h * bytesPerPixel)) {}
    
    ~Framebuffer() { free(data); }

    inline void setPixel(uint32_t w, uint32_t h, const Vec3 &color)
    {
        uint32_t id = bytesPerPixel * (h * width + w);
        data[id + 0] = color.x;
        data[id + 1] = color.y;
        data[id + 2] = color.z;
    }

    // Copies texture into framebuffer
    inline void setPixel(Texture2D &tex)
    {
        if (tex.getWidth() != width || tex.getHeight() != height)
        {
            setSize(tex.getWidth(), tex.getHeight());
        }

        for (uint32_t y = 0; y < height; ++y)
        {
            for (uint32_t x = 0; x < width; ++x)
            {
                auto index = bytesPerPixel * (y * width + x); 
                const Vec3 color = tex.getPixel(index);

                data[index + 0] = 255.99f * color.x;
                data[index + 1] = 255.99f * color.y;
                data[index + 2] = 255.99f * color.z;                
            }
        }
    } 

    inline uint32_t getWidth() const { return width; }
    inline uint32_t getHeight() const { return height ;}

    inline void setSize(uint32_t w, uint32_t h)
    {
        width = w;
        height = h;

        if (!data)
        {
            data = (unsigned char*)malloc(w * h * bytesPerPixel);
        }
        else
        {
            data = (unsigned char*)realloc(data, w * h * bytesPerPixel);
        }
    }

    inline void writeToPNGFile(const char *fileName)
    {
        stbi_flip_vertically_on_write(true);
        stbi_write_png(fileName, width, height, bytesPerPixel, data, width * bytesPerPixel);
    }

private:
    unsigned char *data;
    uint32_t width;
    uint32_t height;
};

class Camera
{
public:
    Camera() {}
    Camera(float fov, float aspect, const Vec3& lookPos, const Vec3 &lookAt, const Vec3& up) : position(lookPos)
    {
        const float fovRad = fov * M_PI / 180;
        const float hHeight = tanf(fovRad / 2.0f);
        const float hWidth = hHeight * aspect;
        
        const Vec3 z = Vec3::normalize(lookPos - lookAt);
        const Vec3 x = Vec3::normalize(Vec3::cross(up, z));
        const Vec3 y = Vec3::normalize(Vec3::cross(z, x));
        
        screenLowerLeft = z * (-1.0f) - hWidth * x - hHeight * y;
        wVec = x * 2.0f * hWidth;
        hVec = y * 2.0f * hHeight;
    }

    inline Ray getRay(float u, float v)
    {
        const float fov = M_PI / 3.0f;
        const Vec3 dir = screenLowerLeft + u * wVec + v * hVec - position;
        return { position, dir };
    }

private:
    Vec3 position;
    Vec3 screenLowerLeft;
    Vec3 wVec;
    Vec3 hVec;    
};

class HitTable
{
public:
    HitTable(const std::initializer_list<Hittable*> &l) : m_objects(std::move(l)) { }

    inline void addObject(std::shared_ptr<Hittable> &obj)
    {
        //m_objects.emplace_back(obj);
    }

    inline bool raycast(const Vec3 &rayO, const Vec3 &rayD, float tmin, float tmax, HitEntry &entry)
    {
        bool hitAny = false;
        float closest = tmax;
        for (const auto &obj : m_objects)
        {
            bool isHit = obj->isHit(rayO, rayD, tmin, closest, entry);

            if (isHit)
            {
                closest = entry.t;
            }

            hitAny |= isHit;
        }

        return hitAny;
    }

    inline bool traceBVH(AABB &out)
    {
        if (m_objects.empty())
        {
            return false;
        }

        AABB firstHittable;
        bool firstBox = true;

        for (const auto &obj : m_objects)
        {
            if (!obj->calculateBox(firstHittable))
            {
                return false;
            }   

            out = firstBox ? firstHittable : AABB::calculateSurround(out, firstHittable);
            firstBox = false;
        }

        return true;
    }
    
    std::vector<Hittable*> getObjects() const { return m_objects; }


private:
    std::shared_ptr<BVHNode> m_root;
    std::vector<Hittable*> m_objects;
};

constexpr static int maxDepth = 50;

//  Threading Tests
//  1280 * 720, 4 threads, ~35 sec
//  1280 * 720, 8 threads, ~34 sec
//  1920 * 1080, 4 threads, ~80 sec
//  1920 * 1080, 8 threads, ~75 sec

static Mesh *cerverus = OBJLoader::load("res/cerberus.obj", { -1.0f, 0.0f, -2.0f } );
static Mesh* cube = OBJLoader::load("res/cube.obj", { 1.0f, 0.0f, -2.0f });

static Texture2D canada("res/canada.png");

Vec3 getColor(const Ray &ray, int depth)
{
    static DiffuseMaterial diffuseRed = { Vec3(1.0f, 0.0f, 0.0f) }; 
    static DiffuseMaterial diffuseBlue = { Vec3(0.0f, 0.0f, 1.0f) };
    static DiffuseMaterial diffuseGreen = { Vec3(0.0f, 1.0f, 0.0f) };
    static DiffuseMaterial diffuseGray = { Vec3(0.8f, 0.8f, 0.8f) };
    static MetalMaterial metalMaterial = { Vec3(0.7f, 0.2f, 0.0f), 0.0f };
    static MetalMaterial grayMaterial = { Vec3(0.7f, 0.7f, 0.7f), 1.0f };
    static DiffLight lightWhite = { Vec3(1.0f, 1.0f, 1.0f) };
    
    static IsotropicMaterial albedoMat = { canada };

    static float R = cosf(M_PI / 4.0f);

    static HitTable scene =
    {
        //std::make_shared<Sphere>(Vec3(0.0f, 0.5f, -6.0f), 1.5f, &diffuseRed).get(),
        //std::make_shared<Sphere>(Vec3(0.0f, -201.5f, -5.0f), 200.0f, &diffuseGray).get(),
        //std::make_shared<Sphere>(Vec3(6.0f,  0.5f, -5.0f), 1.5f, &metalMaterial).get(),
        //std::make_shared<Sphere>(Vec3(-6.0f, 0.5f, -5.0f), 1.5f, &grayMaterial).get(),
        //std::make_shared<Sphere>(Vec3(0.0f, 3.0f, -5.0f), 0.5f, &lightWhite).get()
        new Sphere(Vec3(0.0f, 0.5f, -6.0f), 1.5f, &diffuseBlue),
        new Sphere(Vec3(-1.0f, 0.01f, -2.0f), 0.5f, &albedoMat),
        new Sphere(Vec3(0.0f, -201.5f, -5.0f), 200.0f, &diffuseGray),
        new Sphere(Vec3(6.0f,  0.5f, -5.0f), 1.5f, &metalMaterial),
        new Sphere(Vec3(-6.0f, 0.5f, -5.0f), 1.5f, &grayMaterial),
        new Sphere(Vec3(0.0f, 3.0f, -5.0f), 0.5f, &lightWhite),
        //new Mesh(cerverus, &diffuseGreen)

        //new BBox(Vec3(-3.0f, 0.0f, -2.0f), Vec3(3.0f, 2.0f, -4.0f), &diffuseGreen),
    };

    //static HitTable BVH = {};
    //BVH.addObject(std::make_shared<BVHNode>(scene.getObjects(), 0, 1));

    /*static HitTable scene = 
    {
        { Vec3(0.0f, 0.0f, -1.0f), 0.5f, &diffuseRed },
        { Vec3(0.0f, -100.5f, -1.0f), 100.0f, &diffuseGray },
        { Vec3(1.0f,  0.0f, -1.0f), 0.5f, &metalMaterial },
        { Vec3(-1.0f, 0.0f, -1.0f), 0.5f, &grayMaterial }
    };*/
    
    //?????????????
    AABB out = {};
    //scene.traceBVH(out);

    const Vec3 &rayOrig = ray.getOrigin();
    const Vec3 &rayDir = ray.getDir();
    
    HitEntry sceneEntry = {};

    /*if (depth >= maxDepth && cube->isHit(rayOrig, rayDir, tMin, tMax, sceneEntry))
    {       
        return { 1.0f, 0.0f, 0.0f };
    }*/
    
    if (depth >= maxDepth)
    {
        return { 0.0f, 0.0f, 0.0f };
    }

    if (scene.raycast(rayOrig, rayDir, 0.0001f, 1000.0f, sceneEntry))
    {
        Vec3 attn;
        Ray scatterd;
        Vec3 emittedColor = sceneEntry.m->emitted(sceneEntry.u, sceneEntry.v);

        if (!sceneEntry.m->scatter(rayOrig, rayDir, attn, sceneEntry, scatterd))
        {
            return emittedColor;
        }

        return emittedColor + attn * getColor(scatterd, depth + 1);
    }

    const float t = 0.5f * (rayDir.y + 1.0f);
    Vec3 skyColor = (1.0f - t) * Vec3(1.0f, 1.0f, 1.0f) + t * Vec3(0.5f, 0.7f, 1.0f);
   
    return skyColor;
}

int main()
{
    uint32_t threadCount = std::thread::hardware_concurrency();
    
    if (threadCount > 8)
    {
        std::cout << "Threads supported: " << threadCount << "\n";
        std::cout << "Executing...\n";        
    }
    else
    {
        std::cout << "Failed to create 8 threads\n";
        return 0;
    }

    const char* filename = "outMeshTest.png";
    constexpr static uint32_t numSamples = 100;
    constexpr static float invNumSamples = 1.0f / numSamples;

    //Mesh *cubeMesh = OBJLoader::load("res/cube.obj");

    Framebuffer screen(2000, 1000);
    float as = screen.getWidth() / screen.getHeight();
    Camera cam(90.0f, as, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f });
    
    Texture2D whiteTex;

    whiteTex.setSize(screen.getWidth(), screen.getHeight());
    whiteTex.white();

    Texture2D redTex;
    redTex.setSize(screen.getWidth(), screen.getHeight());
    redTex.color( { 1.0f, 0.0f, 0.0f } );

    auto tileWorker = [&screen, &cam](uint32_t xT, uint32_t yT, uint32_t w, uint32_t h) 
    {
        for (uint32_t y = yT; y < yT + h; ++y)
        {
            for (uint32_t x = xT; x < xT + w; ++x)
            {
                Vec3 weightedColor = { 0.0f, 0.0f, 0.0f };
                
                for (uint32_t s = 0; s < numSamples; ++s)
                {
                    auto u = (float(x) + randf()) / (float)(screen.getWidth());
                    auto v = (float(y) + randf()) / (float)(screen.getHeight());
                    weightedColor += getColor(cam.getRay(u, v), 0);
                }
                
                weightedColor *= invNumSamples;
                screen.setPixel(x, y, Vec3(255.99f * sqrt(weightedColor.x), 255.99f * sqrt(weightedColor.y), 255.99f * sqrt(weightedColor.z)));
            }
        }
    };
 
    uint32_t tileSizeX = screen.getWidth() / 2;
    uint32_t tileSizeY = screen.getHeight() / 4;

    auto timeStart = std::chrono::system_clock::now();

    //tileWorker(0, 0, screen.getWidth(), screen.getHeight());

    std::thread th1([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(0, 0, tileSizeX, tileSizeY); });
    std::thread th2([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(tileSizeX, 0, tileSizeX, tileSizeY); });
    std::thread th3([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(0, tileSizeY, tileSizeX, tileSizeY); });
    std::thread th4([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(tileSizeX, tileSizeY, tileSizeX, tileSizeY); });
    
    std::thread th5([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(0, tileSizeY + tileSizeY, tileSizeX, tileSizeY); });
    std::thread th6([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(tileSizeX, tileSizeY + tileSizeY, tileSizeX, tileSizeY); });
    std::thread th7([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(0, tileSizeY + tileSizeY + tileSizeY, tileSizeX, tileSizeY); });
    std::thread th8([&tileWorker, &tileSizeX, &tileSizeY]() { tileWorker(tileSizeX, tileSizeY + tileSizeY + tileSizeY, tileSizeX, tileSizeY); });
    
    th1.join();
    th2.join();
    th3.join();
    th4.join();
    
    th5.join();
    th6.join(); 
    th7.join();
    th8.join();

    auto timeEnd = std::chrono::system_clock::now();
    std::chrono::duration<double> timeElapsed = timeEnd - timeStart;
    std::cout << "Finished writing. Time elapsed: " << timeElapsed.count() << "\n";

    screen.writeToPNGFile(filename);
    
    return 0;
}
