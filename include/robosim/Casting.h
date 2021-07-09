/**
 * @file Casting.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __CASTING_H__
#define __CASTING_H__

#include <memory>

namespace typecasting {

template <typename T> auto cast(void *ptr) { return static_cast<T>(ptr); }

template <typename T> auto cast_ptr(std::shared_ptr<void> ptr) {
    return std::static_pointer_cast<T>(ptr);
}

template <typename T, typename... Args> auto make_ptr(Args... args) {
    return std::make_shared<T>(args...);
}

} // namespace typecasting

#endif // !__CASTING_H__
