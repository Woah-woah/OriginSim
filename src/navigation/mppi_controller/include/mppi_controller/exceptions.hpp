// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#ifndef MPPI_CONTROLLER__EXCEPTIONS_HPP_
#define MPPI_CONTROLLER__EXCEPTIONS_HPP_
#include <stdexcept>
namespace mppi
{
class ControllerException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
class InvalidPath : public ControllerException
{
public:
  using ControllerException::ControllerException;
};
class ControllerTFError : public ControllerException
{
public:
  using ControllerException::ControllerException;
};
class NoValidControl : public ControllerException
{
public:
  using ControllerException::ControllerException;
};
}  // namespace mppi
#endif  // MPPI_CONTROLLER__EXCEPTIONS_HPP_
