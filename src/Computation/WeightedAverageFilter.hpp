#pragma once

#include <tuple>
#include <vector>
#include <string>

#include <Dense>

namespace Computation {

template <class T>
class WeightedAverageFilter {
private:
    virtual T Add(T current, T differential) = 0;
protected:
    T _initialValue;
    std::vector<std::tuple<std::string, T, long long, double>> _filter;
public:
    WeightedAverageFilter(T initialValue) : _initialValue(initialValue) {}

    virtual void AddAbsoluteValue(std::string id, T raw, long long time, double weight) {
        bool found = false;
        for (auto &fv : _filter) {
            auto i = std::get<0>(fv);
            if (i == id) {
                if (time > std::get<2>(fv))
                    fv = std::make_tuple(id, raw, time, weight);
                found = true;
                break;
            }
        }
        if (!found) _filter.push_back(std::make_tuple(id, raw, time, weight));
    }

    virtual void AddDifferentialValue(std::string id, T raw, long long time, double weight) {
        bool found = false;
        T val = Add(GetFilteredValue(id), raw);
        for (auto &fv : _filter) {
            auto i = std::get<0>(fv);
            if (i == id) {
                fv = std::make_tuple(id, val, time, weight);
                found = true;
                break;
            }
        }
        if (!found) _filter.push_back(std::make_tuple(id, val, time, weight));
    }

    virtual void Clear(T initialValue) {
        _filter.clear();
        _initialValue = initialValue;
    }

    virtual T GetFilteredValue(std::string id = "") = 0;
};

class WeightedAverageVectorFilter : public WeightedAverageFilter<Eigen::Vector3d> {
private:
    virtual Eigen::Vector3d Add(Eigen::Vector3d current, Eigen::Vector3d differential) override {
        return current + differential;
    }
public:
    WeightedAverageVectorFilter(Eigen::Vector3d initialValue) : WeightedAverageFilter(initialValue) {}

    virtual Eigen::Vector3d GetFilteredValue(std::string id = "") override {
        if (_filter.size() < 1)
            return _initialValue;
        Eigen::Vector3d total;
        double weight;
        if (id.empty()) {
            total = Eigen::Vector3d(0,0,0);
            weight = 0.0d;
        } else {
            total = _initialValue;
            weight = 1.0d;
        }
        for (unsigned int i = 0; i < _filter.size(); i++) {
            auto fv = _filter[i];
            if (id.empty()) {
                auto v = std::get<1>(fv);
                auto w = std::get<3>(fv);
                (i == 0) ? total = v * w : total += v * w;
                weight += w;
            }
            else if (std::get<0>(fv) == id) {
                return std::get<1>(fv);
            }
        }
        return total / weight;
    };
};

class WeightedAverageQuaternionFilter : public WeightedAverageFilter<Eigen::Quaterniond> {
private:
    virtual Eigen::Quaterniond Add(Eigen::Quaterniond current, Eigen::Quaterniond differential) {
        return current * differential;
    }
public:
    WeightedAverageQuaternionFilter(Eigen::Quaterniond initialValue) : WeightedAverageFilter(initialValue) {}

   virtual Eigen::Quaterniond GetFilteredValue(std::string id = "") override {
        if (_filter.size() < 1)
            return _initialValue;
        Eigen::Quaterniond total = id.empty() ? std::get<1>(_filter[0]) : _initialValue;
        for (unsigned int i = 1; i < _filter.size(); i++) {
            auto fv = _filter[i];
            if (id.empty()) {
                auto v = std::get<1>(fv);
                auto w = std::get<3>(fv);
                total = total.slerp(w, v);
            }
            else if (std::get<0>(fv) == id) {
                return std::get<1>(fv);
            }
        }
        return total;
   }
};

};
