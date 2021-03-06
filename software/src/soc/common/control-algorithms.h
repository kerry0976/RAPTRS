/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#pragma once

#include "generic-function.h"
#include <Eigen/Dense>

class __PID2Class {
  public:
    void Configure(float Kp, float Ki, float Kd, float Tf, float b, float c, float Min, float Max);
    void Run(GenericFunction::Mode mode, float Reference, float Feedback, float dt, float *y, float *ff, float *fb);
    void Clear();
  private:
    uint8_t mode_ = GenericFunction::Mode::kStandby;

    float Kp_, Ki_, Kd_, Tf_, b_, c_, y_, ff_, fb_, Min_, Max_;
    bool initLatch_ = false;

    float ffProportionalError_, ffDerivativeError_, ffIntegralError_, ffDerivativeErrorState_;
    float fbProportionalError_, fbDerivativeError_, fbIntegralError_, fbDerivativeErrorState_;
    float ffPreviousDerivativeError_, ffPreviousDerivativeErrorState_, ffIntegralErrorState_;
    float fbPreviousDerivativeError_, fbPreviousDerivativeErrorState_, fbIntegralErrorState_;

    void InitializeState(float ff, float fb);
    void ComputeDerivative(float dt);
    void UpdateState(float dt);
    void CalculateCommand();
    void Reset();
};

class __SSClass {
  public:
    void Configure(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, float dt, Eigen::VectorXf Min, Eigen::VectorXf Max);
    void Run(GenericFunction::Mode mode, Eigen::VectorXf u, float dt, Eigen::VectorXf *y);
    void Clear();
  private:
    uint8_t mode_ = GenericFunction::Mode::kStandby;

    Eigen::MatrixXf A_, B_, C_, D_;
    Eigen::VectorXf x_;
    Eigen::VectorXf y_, Min_, Max_;

    bool initLatch_ = false;

    Eigen::MatrixXf CA_inv_, CB_;

    void InitializeState(Eigen::VectorXf u, Eigen::VectorXf y, float dt);
    void UpdateState(Eigen::VectorXf u, float dt);
    void OutputEquation(Eigen::VectorXf u, float dt);
    void Reset();
};
