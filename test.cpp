#include <iostream>
#include <vector>
#include <map>
#include <string>

using namespace std;

//2.0.車共通 基底構造体
struct Vehicle
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
public:
    Vehicle(double sx, double sy, double gx, double gy) : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0) {}

    double calcDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
        return dis_;
    }

    double getDistance() const{
        return dis_;
    }

};

//2.1.自動車(送迎含む)
struct Car : Vehicle
{
private:
    double park_;//駐車代
    double fuelpk_;//キロ当たり燃費
    double fuel_;//燃費
    double cv_;//自動車速度
public:
    //駐車代は重回帰などにより直接求めるのを避けたいが…
    Car(double sx, double sy, double gx, double gy, double p, double fp, double cv)
        : Vehicle(sx, sy, gx, gy), park_(p), fuelpk_(fp), fuel_(0), cv_(cv) {}

    double calcFuel(){
        fuel_ = dis_ * fuelpk_;
        return fuel_;
    }

    double getFuel() const{
        return fuel_;
    }
    double getCost() const{
        return park_ + fuel_;
    }
    double getTime() const{
        return (double)dis_ / cv_;
    }
};

//2.2.タクシー
struct Taxi : Vehicle
{
private:
    //std::map farebd_(fare by distance)は、料金が変わる距離とその金額を格納fare[0] = 100 fare[0.255] = 270,...
    map<double, double> farebd_;
    double fare_;
    double tv_;//タクシー速度
public:
    Taxi(double sx, double sy, double gx, double gy, map<double, double>& farebd, double tv)
        : Vehicle(sx, sy, gx, gy), farebd_(farebd), fare_(0), tv_(tv) {}

    double getCost(){
        double distance = calcDistance();

        //mapの各farebd_の要素について
        for(const auto& i : farebd_){
            fare_ = i.second;
            if(distance < i.first){
                //「次の上がり幅」よりも小さかったら、その時のタクシー金額を出力
                return i.second;
            }
        }
    }
    double getTime() const{
        return (double)dis_ / tv_;
    }
};



int main()
{
    cout << "Hello" << endl;
    //1.個人の特性の入力
    int n;
    vector<map<string, double>> feat(n);
    //2.各交通手段 基礎情報
    //2.1.自動車
    //2.1.1.駐車場代 将来的には関数
    int park = 0;
    //2.1.2.燃費
    double fuel = 0;
}