#include "ConductorModalClassifier.h"

#include "DataStructure/Structure/StructureData.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace SolverNameSpace
{
class ConductorModalMetricBuilder
{
    using Row = std::map<int, double>;
    using Triplets = std::vector<Eigen::Triplet<double>>;

    struct DisjointNodes
    {
        std::map<int, int> parent;
        int Find(int node)
        {
            auto [it, inserted] = parent.emplace(node, node);
            if (inserted || it->second == node)
                return node;
            return it->second = Find(it->second);
        }
        void Join(int first, int second)
        {
            first = Find(first);
            second = Find(second);
            if (first != second)
                parent[second] = first;
        }
    };

    struct CableData
    {
        std::shared_ptr<ElementCable> element;
        std::array<std::shared_ptr<Node>, 2> nodes;
        double lineMass = 0.0;
        double rotaryMass = 0.0;
        double referenceLength = 0.0;
    };

    struct WirePath
    {
        int wireId = -1;
        int bundleCount = 0;
        int component = -1;
        std::vector<std::shared_ptr<Node>> nodes;
        std::vector<const CableData*> elements;
    };

    static Eigen::Vector3d Position(const Node& node, bool current)
    {
        Eigen::Vector3d value(node.m_X, node.m_Y, node.m_Z);
        if (current)
            for (int component = 0; component < 3; ++component)
                if (node.m_Displacement.size() > static_cast<std::size_t>(component))
                    value[component] += node.m_Displacement[static_cast<std::size_t>(component)];
        return value;
    }

    static bool Before(const Node& first, const Node& second)
    {
        const Eigen::Vector3d a = Position(first, false);
        const Eigen::Vector3d b = Position(second, false);
        for (int component = 0; component < 3; ++component)
            if (a[component] != b[component])
                return a[component] < b[component];
        return first.m_Id < second.m_Id;
    }

    static int FreeDof(const Node& node, int component, int fixedDofs, int freeDofs)
    {
        if (component >= node.m_DOF.size())
            return -1;
        const int index = node.m_DOF[component] - fixedDofs;
        return index >= 0 && index < freeDofs ? index : -1;
    }

    static void Add(_OUT Row& row, int index, double value)
    {
        if (index >= 0 && value != 0.0)
            row[index] += value;
    }

    static void Outer(const Row& row, double weight, _OUT Triplets& triplets)
    {
        if (weight <= 0.0)
            return;
        for (const auto& [first, firstValue] : row)
            for (const auto& [second, secondValue] : row)
                if (firstValue != 0.0 && secondValue != 0.0)
                    triplets.emplace_back(first, second, weight * firstValue * secondValue);
    }

    static void TranslationRows(const std::shared_ptr<Node>& first, const std::shared_ptr<Node>& second,
                                double fraction, const Eigen::Matrix3d& projector, int fixedDofs, int freeDofs,
                                _OUT std::array<Row, 3>& rows)
    {
        for (int output = 0; output < 3; ++output)
            for (int input = 0; input < 3; ++input)
            {
                Add(rows[static_cast<std::size_t>(output)], FreeDof(*first, input, fixedDofs, freeDofs),
                    (1.0 - fraction) * projector(output, input));
                Add(rows[static_cast<std::size_t>(output)], FreeDof(*second, input, fixedDofs, freeDofs),
                    fraction * projector(output, input));
            }
    }

    static void LocalMetrics(const CableData& cable, int fixedDofs, int freeDofs, bool includeTransverse,
                             _OUT Triplets& transverse, _OUT Triplets& twist)
    {
        const Eigen::Vector3d chord = Position(*cable.nodes[1], true) - Position(*cable.nodes[0], true);
        const Eigen::Vector3d axis = chord.normalized();
        const Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - axis * axis.transpose();
        if (includeTransverse)
        {
            constexpr double point = 0.28867513459481288225;
            for (double fraction : {0.5 - point, 0.5 + point})
            {
                std::array<Row, 3> rows;
                TranslationRows(cable.nodes[0], cable.nodes[1], fraction, projector, fixedDofs, freeDofs, rows);
                for (const Row& row : rows)
                    Outer(row, 0.5 * cable.lineMass * cable.referenceLength, transverse);
            }
        }
        if (cable.rotaryMass <= 0.0)
            return;

        Eigen::VectorXd gradient;
        cable.element->GetMeanTwistDerivatives(gradient);
        std::vector<int> counts;
        cable.element->GetNodeLocalDOFCounts(counts);
        Row mean;
        Row difference;
        int offset = 0;
        for (int endpoint = 0; endpoint < 2; ++endpoint)
        {
            const auto& node = cable.nodes[static_cast<std::size_t>(endpoint)];
            for (int direction = 0; direction < counts[static_cast<std::size_t>(endpoint)]; ++direction)
            {
                const int index = FreeDof(*node, direction, fixedDofs, freeDofs);
                Add(mean, index, gradient[offset + direction]);
                // 转动部分恢复两端轴向扭角的半差；已提交静力参考态下与节点插值一致，
                // 同时避免交替高阶扭转在单元中点相消后被漏判。
                if (direction >= 3)
                    Add(difference, index, (endpoint == 0 ? 1.0 : -1.0) * gradient[offset + direction]);
            }
            offset += counts[static_cast<std::size_t>(endpoint)];
        }
        Outer(mean, cable.rotaryMass * cable.referenceLength, twist);
        Outer(difference, cable.rotaryMass * cable.referenceLength / 3.0, twist);
    }

    static bool MakeWirePath(const std::vector<const CableData*>& cables, int component, int wireId,
                             _OUT WirePath& path)
    {
        std::map<int, std::vector<const CableData*>> adjacent;
        std::map<int, std::shared_ptr<Node>> nodes;
        for (const CableData* cable : cables)
        {
            if (cable->element->m_AeroBundleCount > 0)
            {
                if (path.bundleCount > 0 && path.bundleCount != cable->element->m_AeroBundleCount)
                    return false;
                path.bundleCount = cable->element->m_AeroBundleCount;
            }
            for (const auto& node : cable->nodes)
            {
                adjacent[node->m_Id].push_back(cable);
                nodes[node->m_Id] = node;
            }
        }
        std::shared_ptr<Node> start;
        int endpointCount = 0;
        for (const auto& [id, neighbors] : adjacent)
        {
            if (neighbors.size() > 2)
                return false;
            if (neighbors.size() == 1)
            {
                ++endpointCount;
                if (!start || Before(*nodes[id], *start))
                    start = nodes[id];
            }
        }
        if (endpointCount != 2 || !start)
            return false;
        path.component = component;
        path.wireId = wireId;
        path.nodes.push_back(start);
        const CableData* previous = nullptr;
        while (true)
        {
            const int id = path.nodes.back()->m_Id;
            const CableData* next = nullptr;
            for (const CableData* candidate : adjacent[id])
                if (candidate != previous)
                    next = candidate;
            if (!next)
                break;
            path.elements.push_back(next);
            path.nodes.push_back(next->nodes[next->nodes[0]->m_Id == id ? 1 : 0]);
            previous = next;
            if (path.elements.size() > cables.size())
                return false;
        }
        return path.elements.size() == cables.size();
    }

    static double EndpointDistance(const WirePath& first, const WirePath& second)
    {
        return std::max((Position(*first.nodes.front(), false) - Position(*second.nodes.front(), false)).norm(),
                        (Position(*first.nodes.back(), false) - Position(*second.nodes.back(), false)).norm());
    }

    static bool BundleSpan(const std::vector<WirePath>& paths, const std::vector<std::pair<int, int>>& ranges,
                           int fixedDofs, int freeDofs, _OUT Triplets& bundle, _OUT Triplets& transverse,
                           _OUT std::set<const CableData*>& included, _OUT int& sectionCount)
    {
        Eigen::Vector3d start = Eigen::Vector3d::Zero();
        Eigen::Vector3d end = Eigen::Vector3d::Zero();
        for (std::size_t wire = 0; wire < paths.size(); ++wire)
        {
            start += Position(*paths[wire].nodes[static_cast<std::size_t>(ranges[wire].first)], false);
            end += Position(*paths[wire].nodes[static_cast<std::size_t>(ranges[wire].second)], false);
        }
        Eigen::Vector3d parameterAxis = end - start;
        if (parameterAxis.norm() <= 1.0e-10)
            return false;
        parameterAxis.normalize();
        struct ParametricWire
        {
            std::vector<double> coordinates;
            int first = 0;
        };
        std::vector<ParametricWire> wires(paths.size());
        std::vector<double> knots;
        double lower = -std::numeric_limits<double>::infinity();
        double upper = std::numeric_limits<double>::infinity();
        for (std::size_t wire = 0; wire < paths.size(); ++wire)
        {
            auto& parameter = wires[wire];
            parameter.first = ranges[wire].first;
            for (int index = ranges[wire].first; index <= ranges[wire].second; ++index)
                parameter.coordinates.push_back(
                    parameterAxis.dot(Position(*paths[wire].nodes[static_cast<std::size_t>(index)], false)));
            for (std::size_t index = 1; index < parameter.coordinates.size(); ++index)
                if (parameter.coordinates[index] - parameter.coordinates[index - 1] <= 1.0e-10)
                    return false;
            lower = std::max(lower, parameter.coordinates.front());
            upper = std::min(upper, parameter.coordinates.back());
            knots.insert(knots.end(), parameter.coordinates.begin(), parameter.coordinates.end());
        }
        if (upper - lower <= 1.0e-10)
            return false;
        std::sort(knots.begin(), knots.end());
        knots.erase(std::unique(knots.begin(), knots.end(),
                                [](double a, double b)
                                {
                                    return std::abs(a - b) < 1.0e-10;
                                }),
                    knots.end());
        constexpr std::array<double, 4> gauss{-0.8611363115940525752, -0.3399810435848562648, 0.3399810435848562648,
                                              0.8611363115940525752};
        constexpr std::array<double, 4> weights{0.3478548451374538574, 0.6521451548625461426, 0.6521451548625461426,
                                                0.3478548451374538574};
        Triplets localBundle;
        Triplets localTransverse;
        std::set<const CableData*> localIncluded;
        int localSections = 0;
        for (std::size_t interval = 1; interval < knots.size(); ++interval)
        {
            const double left = std::max(lower, knots[interval - 1]);
            const double right = std::min(upper, knots[interval]);
            if (right - left <= 1.0e-10)
                continue;
            for (std::size_t point = 0; point < gauss.size(); ++point)
            {
                const double coordinate = 0.5 * (left + right + (right - left) * gauss[point]);
                const double weight = 0.5 * (right - left) * weights[point];
                std::vector<Eigen::Vector3d> positions(paths.size());
                std::vector<double> mass(paths.size());
                std::vector<double> fractions(paths.size());
                std::vector<int> indices(paths.size());
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
                double totalMass = 0.0;
                for (std::size_t wire = 0; wire < paths.size(); ++wire)
                {
                    const auto& parameter = wires[wire];
                    const int segment = static_cast<int>(std::upper_bound(parameter.coordinates.begin(),
                                                                          parameter.coordinates.end(), coordinate) -
                                                         parameter.coordinates.begin()) -
                                        1;
                    indices[wire] = parameter.first + segment;
                    const double length = parameter.coordinates[static_cast<std::size_t>(segment + 1)] -
                                          parameter.coordinates[static_cast<std::size_t>(segment)];
                    fractions[wire] = (coordinate - parameter.coordinates[static_cast<std::size_t>(segment)]) / length;
                    const auto& path = paths[wire];
                    const auto* cable = path.elements[static_cast<std::size_t>(indices[wire])];
                    const Eigen::Vector3d first = Position(*path.nodes[static_cast<std::size_t>(indices[wire])], true);
                    const Eigen::Vector3d second =
                        Position(*path.nodes[static_cast<std::size_t>(indices[wire] + 1)], true);
                    positions[wire] = (1.0 - fractions[wire]) * first + fractions[wire] * second;
                    mass[wire] = cable->lineMass * cable->referenceLength / length;
                    totalMass += mass[wire];
                    center += mass[wire] * positions[wire];
                    tangent += mass[wire] * (second - first) / length;
                    localIncluded.insert(cable);
                }
                if (totalMass <= 0.0 || tangent.norm() <= 1.0e-12)
                    return false;
                center /= totalMass;
                tangent.normalize();
                const Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - tangent * tangent.transpose();
                Row angleNumerator;
                double polarMass = 0.0;
                for (std::size_t wire = 0; wire < paths.size(); ++wire)
                {
                    const Eigen::Vector3d direction = tangent.cross(positions[wire] - center);
                    polarMass += mass[wire] * direction.squaredNorm();
                    const int index = indices[wire];
                    const auto& first = paths[wire].nodes[static_cast<std::size_t>(index)];
                    const auto& second = paths[wire].nodes[static_cast<std::size_t>(index + 1)];
                    std::array<Row, 3> projected;
                    TranslationRows(first, second, fractions[wire], projector, fixedDofs, freeDofs, projected);
                    for (int component = 0; component < 3; ++component)
                    {
                        Outer(projected[static_cast<std::size_t>(component)], weight * mass[wire], localTransverse);
                        for (const auto& [dof, coefficient] : projected[static_cast<std::size_t>(component)])
                            Add(angleNumerator, dof, mass[wire] * direction[component] * coefficient);
                    }
                }
                if (polarMass <= 1.0e-16 * totalMass)
                    return false;
                Outer(angleNumerator, weight / polarMass, localBundle);
                ++localSections;
            }
        }
        if (localSections == 0)
            return false;
        bundle.insert(bundle.end(), localBundle.begin(), localBundle.end());
        transverse.insert(transverse.end(), localTransverse.begin(), localTransverse.end());
        included.insert(localIncluded.begin(), localIncluded.end());
        sectionCount += localSections;
        return true;
    }

public:
    static bool Build(const StructureData& data, int fixedDofCount, int freeDofCount,
                      _OUT ConductorModalMetrics& metrics, _OUT QString& errorMessage,
                      const std::function<bool(int)>& isStepActive);
};

double ConductorModalMetrics::TorsionFraction(const Eigen::VectorXd& shape,
                                              const Eigen::SparseMatrix<double>& mass) const
{
    if (mass.rows() != shape.size() || mass.cols() != shape.size() || axialTwist.rows() != shape.size() ||
        transverseMotion.rows() != shape.size() || bundleTorsion.rows() != shape.size())
        return 0.0;
    const double modalMass = shape.dot(mass * shape);
    if (!std::isfinite(modalMass) || modalMass <= 0.0)
        return 0.0;
    const double self = std::clamp(shape.dot(axialTwist * shape) / modalMass, 0.0, 1.0);
    const double transverse = shape.dot(transverseMotion * shape);
    double bundle = 0.0;
    if (bundleSectionCount > 0 && transverse > 1.0e-6 * modalMass)
        bundle = std::clamp(shape.dot(bundleTorsion * shape) / transverse, 0.0, 1.0);
    // 同时覆盖束整体扭转与导线自身扭转两种运动机制，不依赖全局 RotX。
    return std::max(self, bundle);
}

bool ConductorModalMetricBuilder::Build(const StructureData& data, int fixedDofCount, int freeDofCount,
                                        _OUT ConductorModalMetrics& metrics, _OUT QString& errorMessage,
                                        const std::function<bool(int)>& isStepActive)
{
    metrics = ConductorModalMetrics{};
    errorMessage.clear();
    if (fixedDofCount < 0 || freeDofCount <= 0)
    {
        errorMessage = QStringLiteral("导线模态分类的自由度布局无效。");
        return false;
    }
    Triplets bundle;
    Triplets transverse;
    Triplets twist;
    DisjointNodes connectivity;
    DisjointNodes suspension;
    std::set<int> suspensionNodeIds;
    std::vector<CableData> cables;
    try
    {
        for (const auto& [elementId, element] : data.m_Elements)
        {
            if (!element)
                continue;
            const bool connect =
                element->m_Role == ElementRole::Conductor || element->m_Role == ElementRole::IntraPhaseSpacer ||
                element->m_Role == ElementRole::TensionHardware || element->m_Role == ElementRole::SuspensionHardware;
            int firstId = -1;
            for (const auto& weakNode : element->m_pNode)
            {
                const auto node = weakNode.lock();
                if (!node)
                    throw std::runtime_error("Conductor classification found an expired element node");
                if (firstId < 0)
                    firstId = node->m_Id;
                else if (connect)
                    connectivity.Join(firstId, node->m_Id);
                if (element->m_Role == ElementRole::SuspensionHardware)
                {
                    suspension.Join(firstId, node->m_Id);
                    suspensionNodeIds.insert(node->m_Id);
                }
            }
            const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
            if (!cable)
                continue;
            if (cable->m_pNode.size() != 2)
                throw std::runtime_error("Conductor classification requires two-node cable elements");
            CableData entry;
            entry.element = cable;
            entry.nodes = {cable->m_pNode[0].lock(), cable->m_pNode[1].lock()};
            connectivity.Join(entry.nodes[0]->m_Id, entry.nodes[1]->m_Id);
            const auto property = cable->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!material || !section)
                throw std::runtime_error("Conductor classification requires cable material and section references");
            entry.lineMass = section->m_Area * material->m_Density;
            double iy = 0.0;
            double iz = 0.0;
            double polar = 0.0;
            section->Calculate_I(iy, iz, polar);
            entry.rotaryMass = section->m_MassInertiaPerLength.x() > 0.0 ? section->m_MassInertiaPerLength.x()
                                                                         : material->m_Density * polar;
            entry.referenceLength = (Position(*entry.nodes[1], false) - Position(*entry.nodes[0], false)).norm();
            const double currentLength = (Position(*entry.nodes[1], true) - Position(*entry.nodes[0], true)).norm();
            if (!std::isfinite(entry.lineMass) || entry.lineMass < 0.0 || !std::isfinite(entry.rotaryMass) ||
                entry.rotaryMass < 0.0 || !std::isfinite(entry.referenceLength) || entry.referenceLength <= 1.0e-12 ||
                !std::isfinite(currentLength) || currentLength <= 1.0e-12)
                throw std::runtime_error("Conductor classification found invalid cable mass or geometry");
            cables.push_back(std::move(entry));
            static_cast<void>(elementId);
        }
        for (const auto& [id, constraint] : data.m_MPCConstraints)
        {
            static_cast<void>(id);
            if (!constraint || (isStepActive && !isStepActive(constraint->m_StepId)) ||
                (!dynamic_cast<RigidOffsetMPCConstraint*>(constraint.get()) &&
                 !dynamic_cast<AxialTwistTieMPCConstraint*>(constraint.get()) &&
                 !dynamic_cast<TranslationalTieMPCConstraint*>(constraint.get())))
                continue;
            const auto ids = constraint->GetNodeIds();
            for (std::size_t index = 1; index < ids.size(); ++index)
                connectivity.Join(ids[0], ids[index]);
        }
        metrics.conductorCount = static_cast<int>(cables.size());
        std::map<std::pair<int, int>, std::vector<const CableData*>> wireElements;
        for (const auto& cable : cables)
            if (cable.element->m_WireId >= 0 && cable.element->m_AeroBundleCount > 1)
                wireElements[{connectivity.Find(cable.nodes[0]->m_Id), cable.element->m_WireId}].push_back(&cable);
        std::map<int, std::vector<WirePath>> groups;
        for (const auto& [key, elements] : wireElements)
        {
            WirePath path;
            if (MakeWirePath(elements, key.first, key.second, path))
                groups[key.first].push_back(std::move(path));
            else
                ++metrics.skippedBundleCount;
        }
        // 直接支撑且没有间隔棒时，各子导线可能互不连通；
        // 在显式分裂数和互异子导线编号约束下，用两端几何距离匹配。
        std::vector<WirePath> orphans;
        for (auto it = groups.begin(); it != groups.end();)
            if (it->second.size() == 1)
            {
                orphans.push_back(std::move(it->second.front()));
                it = groups.erase(it);
            }
            else
                ++it;
        std::set<std::size_t> matched;
        int extraGroup = -1;
        for (std::size_t seed = 0; seed < orphans.size(); ++seed)
        {
            if (matched.count(seed))
                continue;
            std::vector<std::size_t> candidates{seed};
            const auto& reference = orphans[seed];
            const double length =
                (Position(*reference.nodes.back(), false) - Position(*reference.nodes.front(), false)).norm();
            const double tolerance = std::max(1.0e-8, 0.02 * length);
            std::map<int, std::pair<double, std::size_t>> nearestByWire;
            std::set<int> ambiguousWireIds;
            for (std::size_t other = 0; other < orphans.size(); ++other)
            {
                const auto& candidate = orphans[other];
                if (matched.count(other) || candidate.wireId == reference.wireId ||
                    candidate.bundleCount != reference.bundleCount)
                    continue;
                const double distance = EndpointDistance(reference, candidate);
                if (distance >= tolerance)
                    continue;
                const auto found = nearestByWire.find(candidate.wireId);
                if (found == nearestByWire.end() || distance < found->second.first - 1.0e-10)
                {
                    nearestByWire[candidate.wireId] = {distance, other};
                    ambiguousWireIds.erase(candidate.wireId);
                }
                else if (std::abs(distance - found->second.first) <= 1.0e-10)
                    ambiguousWireIds.insert(candidate.wireId);
            }
            std::vector<std::pair<double, std::size_t>> nearest;
            for (const auto& [wireId, candidate] : nearestByWire)
            {
                static_cast<void>(wireId);
                nearest.push_back(candidate);
            }
            std::sort(nearest.begin(), nearest.end());
            const std::size_t needed = static_cast<std::size_t>(reference.bundleCount - 1);
            bool valid = nearest.size() >= needed;
            if (valid && nearest.size() > needed)
                valid = nearest[needed].first - nearest[needed - 1].first > 1.0e-10;
            for (std::size_t index = 0; valid && index < needed; ++index)
            {
                const std::size_t selected = nearest[index].second;
                valid = !ambiguousWireIds.count(orphans[selected].wireId);
                for (std::size_t included : candidates)
                    valid = valid && EndpointDistance(orphans[included], orphans[selected]) < tolerance;
                candidates.push_back(selected);
            }
            if (!valid || candidates.size() != static_cast<std::size_t>(reference.bundleCount))
            {
                ++metrics.skippedBundleCount;
                continue;
            }
            auto& group = groups[extraGroup--];
            for (std::size_t index : candidates)
            {
                matched.insert(index);
                group.push_back(orphans[index]);
            }
        }
        std::map<int, std::vector<Eigen::Vector3d>> suspensionAnchors;
        std::map<int, std::vector<std::shared_ptr<Node>>> stationNodes;
        for (const auto& cable : cables)
            for (const auto& node : cable.nodes)
                if (suspensionNodeIds.count(node->m_Id))
                    stationNodes[suspension.Find(node->m_Id)].push_back(node);
        for (const auto& [station, nodes] : stationNodes)
        {
            static_cast<void>(station);
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto& node : nodes)
                center += Position(*node, false);
            center /= static_cast<double>(nodes.size());
            suspensionAnchors[connectivity.Find(nodes.front()->m_Id)].push_back(center);
        }
        std::set<const CableData*> included;
        for (const auto& [component, paths] : groups)
        {
            const int expected = paths.front().bundleCount;
            std::set<int> wireIds;
            bool valid = paths.size() == static_cast<std::size_t>(expected);
            for (const auto& path : paths)
                valid = valid && path.bundleCount == expected && wireIds.insert(path.wireId).second;
            if (!valid)
            {
                ++metrics.skippedBundleCount;
                continue;
            }
            std::vector<std::vector<int>> cuts(paths.size());
            for (std::size_t wire = 0; wire < paths.size(); ++wire)
            {
                const auto& path = paths[wire];
                cuts[wire] = {0, static_cast<int>(path.nodes.size()) - 1};
                for (const auto& anchor : suspensionAnchors[component])
                {
                    double best = std::numeric_limits<double>::infinity();
                    int nearest = -1;
                    for (std::size_t index = 1; index + 1 < path.nodes.size(); ++index)
                    {
                        const double distance = (Position(*path.nodes[index], false) - anchor).squaredNorm();
                        if (distance < best)
                        {
                            best = distance;
                            nearest = static_cast<int>(index);
                        }
                    }
                    if (nearest >= 0)
                        cuts[wire].push_back(nearest);
                }
                std::sort(cuts[wire].begin(), cuts[wire].end());
                cuts[wire].erase(std::unique(cuts[wire].begin(), cuts[wire].end()), cuts[wire].end());
                valid = valid && cuts[wire].size() == cuts.front().size();
            }
            if (!valid)
            {
                ++metrics.skippedBundleCount;
                continue;
            }
            for (std::size_t span = 1; span < cuts.front().size(); ++span)
            {
                std::vector<std::pair<int, int>> ranges;
                for (const auto& wireCuts : cuts)
                    ranges.emplace_back(wireCuts[span - 1], wireCuts[span]);
                if (BundleSpan(paths, ranges, fixedDofCount, freeDofCount, bundle, transverse, included,
                               metrics.bundleSectionCount))
                    ++metrics.bundleSpanCount;
                else
                    ++metrics.skippedBundleCount;
            }
        }
        for (const auto& cable : cables)
            LocalMetrics(cable, fixedDofCount, freeDofCount, !included.count(&cable), transverse, twist);
    }
    catch (const std::exception& error)
    {
        errorMessage = QStringLiteral("导线物理振型指标构造失败：%1").arg(QString::fromUtf8(error.what()));
        return false;
    }
    for (auto* matrix : {&metrics.bundleTorsion, &metrics.transverseMotion, &metrics.axialTwist})
        matrix->resize(freeDofCount, freeDofCount);
    metrics.bundleTorsion.setFromTriplets(bundle.begin(), bundle.end());
    metrics.transverseMotion.setFromTriplets(transverse.begin(), transverse.end());
    metrics.axialTwist.setFromTriplets(twist.begin(), twist.end());
    metrics.summary = QStringLiteral("按静力构型的导线横向位移识别束整体扭转，按真实轴向扭角和截面惯量识别自身扭转；%1 "
                                     "个索单元，%2 档束截面，%3 个积分截面，%4 个拓扑片段使用局部轴回退。")
                          .arg(metrics.conductorCount)
                          .arg(metrics.bundleSpanCount)
                          .arg(metrics.bundleSectionCount)
                          .arg(metrics.skippedBundleCount);
    return true;
}

bool ConductorModalClassifier::Build(const StructureData& data, int fixedDofCount, int freeDofCount,
                                     _OUT ConductorModalMetrics& metrics, _OUT QString& errorMessage,
                                     const std::function<bool(int)>& isStepActive)
{
    return ConductorModalMetricBuilder::Build(data, fixedDofCount, freeDofCount, metrics, errorMessage, isStepActive);
}
}
