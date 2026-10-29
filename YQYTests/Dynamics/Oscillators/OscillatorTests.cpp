#include <gtest/gtest.h>

#include "Dynamics/Oscillators/OscillatorCases.h"
#include "Support/TestPaths.h"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace
{
enum Column
{
    Time,
    Q,
    V,
    A,
    Dt,
    External,
    Restoring,
    Damping,
    Inertia,
    Residual,
    Plastic,
    Increment,
    Iterations,
    ColumnCount
};
using Row = std::array<double, ColumnCount>;
using History = std::vector<Row>;

static double Number(const QJsonObject& object, const char* key)
{
    const QJsonValue value = object.value(QLatin1String(key));
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error(std::string("Missing or nonfinite reference number: ") + key);
    return value.toDouble();
}

static QJsonObject ReadReference()
{
    const QDir directory(
        QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/Dynamics/Oscillators")));
    QFile file(directory.filePath(QStringLiteral("regression-reference.json")));
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot read persistent Newmark literature reference");
    const QByteArray bytes = file.readAll();
    const QByteArray hash = QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex();
    if (hash != "1cd12d5a541c5f0e73516a08391cf5e0618e33e14a997558cb8304b9a69a2262")
        throw std::runtime_error("Newmark independent reference SHA256 differs from the reviewed reference");
    const QJsonObject reference = QJsonDocument::fromJson(bytes).object();
    if (reference.value("schema").toString() != QStringLiteral("newmark-literature-regression-v1") ||
        reference.value("frozen_plan_sha256").toString() !=
            QStringLiteral("e1e7b14e5592c754319d2492d89f2933a2e78b3535fd6e50700094ca560c3dfd") ||
        reference.value("cases").toArray().size() != 14)
        throw std::runtime_error("Newmark literature reference schema, frozen plan or case count is invalid");
    ::testing::Test::RecordProperty("independent_reference_sha256", hash.constData());
    return reference;
}

static History ReadHistory(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        throw std::runtime_error("Cannot read generated Newmark history");
    const QByteArray header = file.readLine().trimmed();
    if (header != "time,q,v,a,accepted_dt,external_force,restoring_force,damping_force,inertia_force,"
                  "equilibrium_residual,committed_plastic_displacement,step_increment,newton_evaluations")
        throw std::runtime_error("Unexpected Newmark history columns");
    History history;
    while (!file.atEnd())
    {
        const QByteArray line = file.readLine().trimmed();
        if (line.isEmpty())
            continue;
        const QList<QByteArray> fields = line.split(',');
        if (fields.size() != ColumnCount)
            throw std::runtime_error("Incomplete Newmark history row");
        Row row{};
        for (int index = 0; index < ColumnCount; ++index)
        {
            bool valid = false;
            row[index] = fields[index].toDouble(&valid);
            if (!valid || !std::isfinite(row[index]))
                throw std::runtime_error("Nonfinite or invalid Newmark history row");
        }
        history.push_back(row);
    }
    return history;
}

static void CheckCommon(const QJsonObject& run, const QJsonObject& result, const QJsonObject& criteria,
                        double tolerance, const History& history)
{
    const QJsonArray exact = run.value("rows").toArray();
    ASSERT_GE(history.size(), 2u);
    ASSERT_EQ(history.size(), static_cast<std::size_t>(exact.size()));
    ASSERT_TRUE(result.value("solved").toBool());
    ASSERT_TRUE(result.value("reached_duration").toBool());
    ASSERT_TRUE(result.value("strictly_increasing_history").toBool());
    EXPECT_EQ(result.value("status").toString(), QStringLiteral("COMPLETE_UNASSESSED"));
    EXPECT_EQ(result.value("factory_solver_type").toInt(), 1);
    EXPECT_DOUBLE_EQ(Number(result, "beta"), 0.25);
    EXPECT_DOUBLE_EQ(Number(result, "gamma"), 0.5);
    EXPECT_DOUBLE_EQ(Number(result, "requested_step"), Number(run, "step"));
    EXPECT_DOUBLE_EQ(Number(result, "requested_duration"), Number(run, "duration"));
    EXPECT_DOUBLE_EQ(Number(result, "nonlinear_tolerance"), tolerance);
    EXPECT_EQ(Number(result, "rejected_attempts"), 0.0);
    EXPECT_EQ(Number(result, "precision_limited_solves"), 0.0);
    EXPECT_EQ(Number(result, "accepted_steps"), history.size() - 1);
    EXPECT_EQ(Number(result, "normal_newton_accepted_steps"), history.size() - 1);
    EXPECT_EQ(Number(result, "commits"), history.size() - 1);
    EXPECT_TRUE(result.value("fixed_step_sequence_matches_requested").toBool());
    EXPECT_TRUE(result.value("fixed_step_reference_comparison_eligible").toBool());
    const double equilibriumLimit = Number(criteria, "accepted_equilibrium_absolute_limit");
    const double kinematicLimit = Number(criteria, "accepted_Newmark_kinematics_absolute_limit");
    EXPECT_LE(Number(result, "maximum_accepted_equilibrium_residual"), equilibriumLimit);
    EXPECT_DOUBLE_EQ(history.front()[Time], 0.0);
    EXPECT_NEAR(history.back()[Time], Number(run, "duration"), 1.0e-9);
    for (int column : {Q, V, A})
    {
        const char* key = column == Q ? "q" : column == V ? "v" : "a";
        EXPECT_NEAR(history.front()[column], Number(exact.first().toObject(), key), 1.0e-14);
    }
    const double step = Number(run, "step");
    const double stepLimit =
        128.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, Number(run, "duration")) + 1.0e-8 * step;
    for (std::size_t index = 0; index < history.size(); ++index)
    {
        const Row& current = history[index];
        SCOPED_TRACE(::testing::Message() << "frame=" << index << " t=" << current[Time]);
        EXPECT_NEAR(current[Time], Number(exact[static_cast<int>(index)].toObject(), "time"), 1.0e-9);
        EXPECT_LE(std::abs(current[Residual]), equilibriumLimit);
        if (index == 0)
            continue;
        const Row& previous = history[index - 1];
        const double h = current[Time] - previous[Time];
        EXPECT_GT(h, 0.0);
        EXPECT_NEAR(current[Dt], h, stepLimit);
        if (index + 1 == history.size())
            EXPECT_LE(h, step + stepLimit);
        else
            EXPECT_NEAR(h, step, stepLimit);
        EXPECT_NEAR(current[Q] - previous[Q], h * previous[V] + 0.25 * h * h * (previous[A] + current[A]),
                    kinematicLimit);
        EXPECT_NEAR(current[V] - previous[V], 0.5 * h * (previous[A] + current[A]), kinematicLimit);
        EXPECT_NEAR(current[Increment], current[Q] - previous[Q], kinematicLimit);
    }
}

static std::array<double, 3> StateErrors(const QJsonObject& run, const History& history)
{
    const QJsonArray exact = run.value("rows").toArray();
    if (history.size() != static_cast<std::size_t>(exact.size()))
        throw std::runtime_error("Cannot compare incomplete Newmark history");
    std::array<double, 3> errors{};
    for (std::size_t index = 0; index < history.size(); ++index)
    {
        const QJsonObject reference = exact[static_cast<int>(index)].toObject();
        for (int component = 0; component < 3; ++component)
            errors[component] = std::max(
                errors[component], std::abs(history[index][Q + component] - Number(reference, component == 0   ? "q"
                                                                                              : component == 1 ? "v"
                                                                                                               : "a")));
    }
    return errors;
}

static void CheckChopra(const QJsonObject& run, const QJsonObject& result, const QJsonObject& criteria,
                        const History& history)
{
    const bool plastic = run.value("case").toString() == QStringLiteral("chopra-elastoplastic");
    const double mass = Number(run, "mass");
    const double stiffness = Number(run, "stiffness");
    const double damping = Number(run, "damping");
    const double forceLimit = Number(criteria, "Chopra_independent_OpenSees_restoring_force_absolute_limit");
    const double kinematicLimit = Number(criteria, "accepted_Newmark_kinematics_absolute_limit");
    const double equilibriumLimit = Number(criteria, "accepted_equilibrium_absolute_limit");
    EXPECT_DOUBLE_EQ(Number(result, "mass"), mass);
    EXPECT_DOUBLE_EQ(Number(result, "stiffness"), stiffness);
    EXPECT_DOUBLE_EQ(Number(result, "damping_ratio"), 0.05);
    EXPECT_NEAR(Number(result, "damping_coefficient"), damping, 1.0e-15);
    if (plastic)
        EXPECT_DOUBLE_EQ(Number(result, "yield_force"), Number(run, "yield_force"));
    const auto errors = StateErrors(run, history);
    const QJsonArray limits = criteria.value("Chopra_independent_OpenSees_absolute_q_v_a_limits").toArray();
    ASSERT_EQ(limits.size(), 3);
    for (int component = 0; component < 3; ++component)
        EXPECT_LE(errors[component], limits[component].toDouble()) << "state component=" << component;
    const QJsonArray references = run.value("rows").toArray();
    double offset = 0.0;
    int plasticLoading = 0;
    int elasticUnloading = 0;
    for (std::size_t index = 0; index < history.size(); ++index)
    {
        const Row& row = history[index];
        SCOPED_TRACE(::testing::Message() << "frame=" << index << " t=" << row[Time]);
        const double force =
            row[Time] >= 0.0 && row[Time] <= 0.6 ? 10.0 * std::sin(2.0 * std::acos(-1.0) * row[Time] / 1.2) : 0.0;
        const double trial = stiffness * (row[Q] - offset);
        const double restoring = plastic ? std::clamp(trial, -7.5, 7.5) : stiffness * row[Q];
        const double previousOffset = offset;
        if (plastic && std::abs(trial) >= 7.5)
            offset = row[Q] - restoring / stiffness;
        EXPECT_NEAR(row[External], force, forceLimit);
        EXPECT_NEAR(row[Restoring], restoring, forceLimit);
        EXPECT_NEAR(row[Damping], damping * row[V], forceLimit);
        EXPECT_NEAR(row[Inertia], mass * row[A], forceLimit);
        EXPECT_NEAR(row[Plastic], offset, kinematicLimit);
        EXPECT_NEAR(row[Restoring], Number(references[static_cast<int>(index)].toObject(), "restoring_force"),
                    forceLimit);
        EXPECT_NEAR(row[Plastic], Number(references[static_cast<int>(index)].toObject(), "plastic_displacement"),
                    kinematicLimit);
        const double residual = force - restoring - damping * row[V] - mass * row[A];
        EXPECT_LE(std::abs(residual), equilibriumLimit);
        EXPECT_NEAR(row[Residual], residual, equilibriumLimit);
        if (index > 0 && plastic)
        {
            if (offset - previousOffset > kinematicLimit)
                ++plasticLoading;
            const double displacement = row[Q] - history[index - 1][Q];
            if (displacement < -kinematicLimit && std::abs(offset - previousOffset) <= kinematicLimit)
            {
                ++elasticUnloading;
                EXPECT_NEAR(row[Restoring] - history[index - 1][Restoring], stiffness * displacement, forceLimit);
            }
        }
    }
    if (plastic)
    {
        EXPECT_GT(plasticLoading, 0);
        EXPECT_GT(elasticUnloading, 0);
    }
    const QJsonArray original = run.value("official_u_table").toArray();
    ASSERT_EQ(original.size(), static_cast<int>(history.size()) - 1);
    for (const QJsonValue& entry : original)
    {
        const QJsonObject reference = entry.toObject();
        const double time = Number(reference, "time");
        const auto found = std::find_if(history.begin(), history.end(),
                                        [time](const Row& row)
                                        {
                                            return std::abs(row[Time] - time) <= 1.0e-9;
                                        });
        ASSERT_NE(found, history.end());
        EXPECT_NEAR((*found)[Q], Number(reference, "q"),
                    Number(criteria, "official_Chopra_displacement_absolute_limit"));
    }
}

static std::array<double, 2> FirstReturnPeak(const History& history, double period, double amplitude)
{
    double peakTime = std::numeric_limits<double>::infinity();
    double peakValue = 0.0;
    for (std::size_t index = 1; index < history.size(); ++index)
    {
        const Row& first = history[index - 1];
        const Row& last = history[index];
        if (last[Time] <= 0.5 * period || first[Time] >= 1.5 * period)
            continue;
        const double h = last[Time] - first[Time];
        const double delta = last[Q] - first[Q];
        const double c1 = h * first[V];
        const double c2 = 3.0 * delta - h * (2.0 * first[V] + last[V]);
        const double c3 = -2.0 * delta + h * (first[V] + last[V]);
        const double a = 3.0 * c3;
        const double b = 2.0 * c2;
        const double c = c1;
        const double small =
            64.0 * std::numeric_limits<double>::epsilon() * std::max({std::abs(a), std::abs(b), std::abs(c)});
        std::vector<double> roots;
        if (std::abs(a) <= small)
        {
            if (std::abs(b) > small)
                roots.push_back(-c / b);
        }
        else
        {
            const double discriminant = b * b - 4.0 * a * c;
            if (discriminant >= 0.0)
            {
                const double numerator = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
                roots.push_back(numerator / a);
                if (numerator != 0.0)
                    roots.push_back(c / numerator);
            }
        }
        for (double s : roots)
        {
            if (s < 0.0 || s > 1.0)
                continue;
            const double time = first[Time] + s * h;
            const double value = first[Q] + s * (c1 + s * (c2 + s * c3));
            if (time > 0.5 * period && time < 1.5 * period && time < peakTime && value > 0.0 &&
                2.0 * c2 + 6.0 * c3 * s < 0.0)
            {
                peakTime = time;
                peakValue = value;
            }
        }
    }
    if (!std::isfinite(peakTime))
        throw std::runtime_error("Missing first returning positive Hermite peak");
    return {100.0 * (peakTime / period - 1.0), 100.0 * (1.0 - peakValue / amplitude)};
}

struct Refinement
{
    double ratio;
    std::array<double, 3> errors;
};

/** @brief Chopra 有阻尼线性及理想弹塑性原表的工厂 Newmark 全时程验收。
 * @details 输入官方 OpenSees NewmarkIntegrator.tcl：m=.2533、k=10、zeta=.05、Fy=7.5，
 *          半正弦荷载10*sin(2*pi*t/1.2)于.6截断；固定h=.1，线性终点1、弹塑性终点.9。
 *          参考为固定commit的官方原位移表及独立OpenSees3.8执行记录；持久JSON及原plan均核对SHA。
 *          原表u绝对门.001；独立q/v/a门1e-8/1e-8/1e-7，恢复力1e-8；更新式1e-10、平衡1e-9。
 *          检查初值、逐帧力/塑性历史、加载卸载、真实固定步且零拒步及零precision停止。
 *          原表v/a不作为失败判据；仅认证Newmark核及测试专用标量模型，不认证生产材料、梁索或SO3。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
TEST(DampedOscillator, NewmarkFixed)
{
    const QJsonObject reference = ReadReference();
    const QJsonObject criteria = reference.value("criteria").toObject();
    QTemporaryDir output;
    ASSERT_TRUE(output.isValid());
    output.setAutoRemove(false);
    RecordProperty("temporary_output_directory", output.path().toStdString());
    int checked = 0;
    for (const QJsonValue& value : reference.value("cases").toArray())
    {
        const QJsonObject run = value.toObject();
        if (run.value("group").toString() != QStringLiteral("original_table"))
            continue;
        const QString name = run.value("name").toString();
        SCOPED_TRACE(name.toStdString());
        QJsonObject result;
        ASSERT_NO_THROW(result = NewmarkLiterature::RunCase(
                            run.value("case").toString(), QDir(output.path()).filePath(name), Number(run, "step"),
                            Number(run, "duration"), Number(reference, "nonlinear_tolerance")));
        const QJsonObject source = reference.value("sources").toObject().value("chopra").toObject();
        EXPECT_EQ(result.value("source_commit").toString(), source.value("commit").toString());
        EXPECT_EQ(result.value("source_sha256").toString(), source.value("file_sha256").toString());
        const History history = ReadHistory(result.value("response_csv").toString());
        ASSERT_NO_FATAL_FAILURE(CheckCommon(run, result, criteria, Number(reference, "nonlinear_tolerance"), history));
        ASSERT_NO_FATAL_FAILURE(CheckChopra(run, result, criteria, history));
        ++checked;
    }
    EXPECT_EQ(checked, 2);
    output.setAutoRemove(!HasFailure());
}

/** @brief Liang/Mosalam 非线性单摆的原图指标及平均加速度 Newmark 二阶收敛验收。
 * @details 输入theta''+sin(theta)=0、初角.1pi/.5pi、初速0、初加速度-sin(theta0)，无量纲时窗5T。
 *          两幅值各取原文h/T=.04/.08/.10，另取.01/.005/.0025细化；直接调用生产工厂固定Newmark。
 *          参考为独立Jacobi椭圆函数逐帧q/v/a及公开论文图标记；CubicHermite求首个正向返回峰。
 *          原图周期/幅值指标差<=.2百分点；最细归一q/v/a误差各<=.001；两次减半阶数各1.8至2.2。
 *          全部12组必须完整、固定步且无拒步/precision停止，更新式<=1e-10、平衡<=1e-9。
 *          5T和首返回峰为显式验证约定；粗步不要求全局高精度，不认证FE装配、有限转动或气动力。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
TEST(NonlinearPendulum, NewmarkFixed)
{
    const QJsonObject reference = ReadReference();
    const QJsonObject criteria = reference.value("criteria").toObject();
    QTemporaryDir output;
    ASSERT_TRUE(output.isValid());
    output.setAutoRemove(false);
    RecordProperty("temporary_output_directory", output.path().toStdString());
    std::map<QString, std::vector<Refinement>> refinements;
    int checked = 0;
    int figureCases = 0;
    for (const QJsonValue& value : reference.value("cases").toArray())
    {
        const QJsonObject run = value.toObject();
        const QString caseName = run.value("case").toString();
        if (!caseName.startsWith(QStringLiteral("liang-pendulum-")))
            continue;
        const QString name = run.value("name").toString();
        SCOPED_TRACE(name.toStdString());
        QJsonObject result;
        ASSERT_NO_THROW(result = NewmarkLiterature::RunCase(caseName, QDir(output.path()).filePath(name),
                                                            Number(run, "step"), Number(run, "duration"),
                                                            Number(reference, "nonlinear_tolerance")));
        const History history = ReadHistory(result.value("response_csv").toString());
        ASSERT_NO_FATAL_FAILURE(CheckCommon(run, result, criteria, Number(reference, "nonlinear_tolerance"), history));
        const double equilibriumLimit = Number(criteria, "accepted_equilibrium_absolute_limit");
        for (const Row& row : history)
        {
            EXPECT_LE(std::abs(row[A] + std::sin(row[Q])), equilibriumLimit);
            EXPECT_NEAR(row[Restoring], std::sin(row[Q]), equilibriumLimit);
            EXPECT_NEAR(row[Inertia], row[A], equilibriumLimit);
            EXPECT_DOUBLE_EQ(row[External], 0.0);
            EXPECT_DOUBLE_EQ(row[Damping], 0.0);
            EXPECT_NEAR(row[Residual], -std::sin(row[Q]) - row[A], equilibriumLimit);
        }
        auto errors = StateErrors(run, history);
        const QJsonArray scales = run.value("normalization_scales_q_v_a").toArray();
        ASSERT_EQ(scales.size(), 3);
        for (int component = 0; component < 3; ++component)
        {
            ASSERT_GT(scales[component].toDouble(), 0.0);
            errors[component] /= scales[component].toDouble();
        }
        if (run.value("group").toString() == QStringLiteral("paper_figure"))
        {
            const auto metrics = FirstReturnPeak(history, Number(run, "period"), Number(run, "initial_angle"));
            const QJsonObject markers = run.value("paper_markers").toObject();
            const double limit = Number(criteria, "Liang_figure_period_and_amplitude_difference_percentage_points");
            EXPECT_NEAR(metrics[0], Number(markers, "period_elongation_percent"), limit);
            EXPECT_NEAR(metrics[1], Number(markers, "amplitude_decay_percent"), limit);
            ++figureCases;
        }
        else if (run.value("group").toString() == QStringLiteral("additional_refinement"))
            refinements[caseName].push_back({Number(run, "step_ratio"), errors});
        if (Number(run, "step_ratio") == 0.0025)
        {
            const QJsonArray limits = criteria.value("Liang_finest_max_normalized_q_v_a_limits").toArray();
            ASSERT_EQ(limits.size(), 3);
            for (int component = 0; component < 3; ++component)
                EXPECT_LE(errors[component], limits[component].toDouble());
        }
        ++checked;
    }
    EXPECT_EQ(checked, 12);
    EXPECT_EQ(figureCases, 6);
    ASSERT_EQ(refinements.size(), 2u);
    const QJsonArray range = criteria.value("Liang_additional_refinement_order_range").toArray();
    ASSERT_EQ(range.size(), 2);
    for (auto& [name, samples] : refinements)
    {
        SCOPED_TRACE(name.toStdString());
        ASSERT_EQ(samples.size(), 3u);
        std::sort(samples.begin(), samples.end(),
                  [](const Refinement& a, const Refinement& b)
                  {
                      return a.ratio > b.ratio;
                  });
        for (std::size_t index = 1; index < samples.size(); ++index)
        {
            EXPECT_DOUBLE_EQ(samples[index - 1].ratio / samples[index].ratio, 2.0);
            for (int component = 0; component < 3; ++component)
            {
                ASSERT_GT(samples[index].errors[component], 0.0);
                const double order = std::log2(samples[index - 1].errors[component] / samples[index].errors[component]);
                EXPECT_GE(order, range[0].toDouble()) << "q/v/a component=" << component;
                EXPECT_LE(order, range[1].toDouble()) << "q/v/a component=" << component;
            }
        }
    }
    output.setAutoRemove(!HasFailure());
}
}
