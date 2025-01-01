#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace  entity{
enum EnemyState : std::int8_t{
    DEAD,
    PATROLING,
    NOT_PATROLING
};

struct EnemyV1{
    public:
    uint32_t tag;
    EnemyState state;
};
template<typename  T>
struct SwapBackArray{
    std::vector<T> arr;
    void eraseAt(size_t i){
        arr[i] = arr[arr.size()-1];
        arr.pop_back();
    }
    void push_back(T obj){
        arr.push_back(obj);
    }
    size_t size(){
        return arr.size();
    }
    T &operator[](size_t index){
        return arr[index];
    }

};

}
