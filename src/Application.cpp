#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "mod.h"

class Light { //освещение
public:
    Vec3f position;
    float intensity;
    float radius;
    Light(const Vec3f& pos, const float in, const float rad) : position(pos), intensity(in), radius(rad) {};
};

struct Material { //объекты
    Material(const float r, const Vec4f& a, const Vec3f& color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere { //сфера
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f& c, const float r, const Material& m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f& orig, const Vec3f& dir, float& t0) const { //пересечение сферы и луча
        Vec3f L = center - orig;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius) return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

Vec3f reflect(const Vec3f& I, const Vec3f& N) {
    return I - N * 2.f * (I * N);
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float eta_t, const float eta_i = 1.f) { // приломление
    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    if (cosi < 0) return refract(I, -N, eta_i, eta_t); 
    float eta = eta_i / eta_t;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vec3f(1, 0, 0) : I * eta + N * (eta * cosi - sqrtf(k)); 
}

bool scene_intersect(const Vec3f& orig, const Vec3f& dir, const std::vector<Sphere>& spheres, Vec3f& hit, Vec3f& N, Material& material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir * dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    float checkerboard_dist = std::numeric_limits<float>::max(); // доска
    if (fabs(dir.y) > 1e-3) {
        float d = -(orig.y + 4) / dir.y; 
        Vec3f pt = orig + dir * d;
        if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
        }
    }
    return std::min(spheres_dist, checkerboard_dist) < 1000;
}

Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light>& lights, size_t depth = 0) { // учет источников освещения
    Vec3f point, N;
    Material material;
    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material)) { //глубина рекурсии для отражения
        return Vec3f(0.6, 0.7, 0.3);
    }
    float z_dist = orig[2] - point[2];

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();

    Vec3f reflect_orig = point;
    if (reflect_dir * N < 0) { 
        reflect_orig = reflect_orig - N * 1e-3;
    }
    else {
        reflect_orig = reflect_orig + N * 1e-3;
    }
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;

    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    
    float diffuse_light_intensity = 0; // освещение Фонга
    float specular_light_intensity = 0;
    std::vector<Light>::const_iterator q = lights.begin();
    while (q != lights.end()) { // объемный источник
        Vec3f random_light_pos = (*q).position;
        if ((*q).radius > 1e-3) {
            float t = float(rand()) / float(RAND_MAX) * 2 * M_PI;
            float z = float(rand()) / float(RAND_MAX) * 2 * (*q).radius - (*q).radius;
            float r = sqrt((*q).radius - z * z);
            random_light_pos[0] = random_light_pos[0] + r * cos(t);
            random_light_pos[1] = random_light_pos[1] + r * sin(t);
            random_light_pos[2] = random_light_pos[2] + z;
        }

        Vec3f light_dir = (random_light_pos - point).normalize();
        float light_dist = (random_light_pos - point).norm(); //расстояние от точки до света
        Vec3f shadow_orig = point;
        if (light_dir * N < 0) { shadow_orig = shadow_orig - N * 1e-3; } // проверка тени
        else { shadow_orig = shadow_orig + N * 1e-3; } 
        Vec3f shadow_pt, shadow_N;
        Material tmp;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmp) && (shadow_pt - shadow_orig).norm() < light_dist) {
            ++q;
            continue; 
        }

        diffuse_light_intensity += (*q).intensity * std::max(0.f, light_dir * N); //интенсивность зависит от угла между нормалью и направлением света
        specular_light_intensity += powf(std::max(0.f, reflect(light_dir, N) * dir), material.specular_exponent) * (*q).intensity; 
        //отсвет обратно пропорционален углу между направлением взгляда и направлением отраженного свет
        ++q;
    }

    Vec3f diff_part = material.diffuse_color * diffuse_light_intensity * material.albedo[0];
    Vec3f spec_part = Vec3f(1.0, 1.0, 1.0) * specular_light_intensity * material.albedo[1];
    Vec3f reflect_part = reflect_color * material.albedo[2];
    Vec3f refract_part = refract_color * material.albedo[3];

    float fog_intensity = 0.93 * (1 - exp(-z_dist / 25));
    Vec3f fog_color(1.0, 1.0, 1.0);
    Vec3f object_color(diff_part + spec_part + reflect_part + refract_part);
    object_color[0] *= (1 - fog_intensity);
    object_color[1] *= (1 - fog_intensity);
    object_color[2] *= (1 - fog_intensity);
    fog_color[0] *= fog_intensity;
    fog_color[1] *= fog_intensity;
    fog_color[2] *= fog_intensity;

    return object_color + fog_color;

}

void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights) { //отрисовка

    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 2;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            Vec3f color(0.0, 0.0, 0.0);
            for (int i_sub = 0; i_sub < 2; i_sub++) {  // антиалиасинг
                for (int j_sub = 0; j_sub < 2; j_sub++) {
                    float x_jitter = float(rand()) / RAND_MAX - 0.5;
                    float y_jitter = float(rand()) / RAND_MAX - 0.5;
                    float dir_x = (i + x_jitter + 0.5) - width / 2.;
                    float dir_y = -(j + y_jitter + 0.5) + height / 2.;    
                    float dir_z = -height / (2. * tan(fov / 2.));
                    color = color + cast_ray(Vec3f(0, 0, 3), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
                }
            }
            color[0] = color[0] / 4;
            color[1] = color[1] / 4;
            color[2] = color[2] / 4;
            framebuffer[i + j * width] = color;
        }
    }

    std::ofstream ofs;
    ofs.open("./out.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i) {
        float mx = std::max(framebuffer[i][0], std::max(framebuffer[i][1], framebuffer[i][2]));
        if (mx > 1) {
            framebuffer[i] = framebuffer[i] * (1. / mx);
        }
        for (size_t j = 0; j < 3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main() {
    Material      ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
    Material      glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);
    Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
    Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-1.0, 8, -20), 2, ivory));
    spheres.push_back(Sphere(Vec3f(-1.0, 4, -20), 2, glass));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.0, -20), 3, red_rubber));
    spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));
    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5, 1.2));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8, 1.1));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7, 1.0));
    render(spheres, lights);
    return 0;
}