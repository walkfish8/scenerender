#ifndef _RULERMVS_CORE_SINGLETON_H_
#define _RULERMVS_CORE_SINGLETON_H_

#include <mutex>

namespace rulermvs
{
namespace core
{

template <typename T>
class Singleton
{
public:
    template <typename... Args>
    static T* instance(Args&&... args)
    {
        if (instance_ == nullptr)
        {
            std::lock_guard<std::mutex> lock{ lock_ };
            if (instance_ == nullptr)
            {
                // 占位，否则内存释放不能动态执行
                singleton_auto_delete_.nothing_need_to_do();

                // 借助临时指针变量实现线程安全
                T* ptmp   = new T(std::forward<Args>(
                    args)...);
                instance_ = ptmp;
            }
        }
        return instance_;
    }

    static void destroy()
    {
        if (instance_ != nullptr)
        {
            delete instance_;
            instance_ = nullptr;
        }
    }

private:
    static T*         instance_;
    static std::mutex lock_;

    Singleton() = delete;
    ~Singleton() = delete;

    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

    struct _SingletonAutoDelete
    {
        virtual ~_SingletonAutoDelete()
        {
            destroy();
        }

        inline void nothing_need_to_do() const {}
    };
    static _SingletonAutoDelete singleton_auto_delete_;
};

template <typename T>
T* Singleton<T>::instance_ = nullptr;

template <typename T>
std::mutex Singleton<T>::lock_;

template <typename T>
typename Singleton<T>::_SingletonAutoDelete Singleton<T>::singleton_auto_delete_;

}  // namespace core
}  // namespace rulermvs

#endif  // _RULERMVS_CORE_SINGLETON_H_