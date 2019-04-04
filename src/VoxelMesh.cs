using System;
using System.Collections.Generic;
using UnityEngine;

namespace ChupakaberGames.NavMesh3D {

    public class VoxelMesh {

        public Vector3 Size {
            get {
                return _size;
            }
        }

        public Vector3 Origin {
            get {
                return _origin;
            }
        }

        public float LeafSize {
            get {
                return _leafSize;
            }
        }

        public int MaxCorners {
            get {
                return _maxCorners;
            }
            set {
                if (value > 0) {
                    _maxCorners = value;
                }
            }
        }

        // Local properties

        private Vector3 _size;
        private Vector3 _origin;
        private float _leafSize;
        private int[] _graphSize = new int[3];
        private int _maxCorners = 1000;

        // Internal collections

        internal Leaf[] _leafs;
        internal List<LeafNode> _searchingList = new List<LeafNode>(0);
        internal List<Leaf> _resultList = new List<Leaf>(0);
        internal int[] _indexes = new int[26];

        /// <summary> 
        /// Create a navmesh3d from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        /// <param name="origin">Origin - minor corner of NavMesh3D space in unity units. Default is -0.5 of size</param>
        /// <param name="leafSize">Size of NavMesh3D minimum node in unity units.</param>
        public void Bake(Vector3 size, Vector3 origin, float leafSize) {
            if (leafSize <= 0.0f) {
                throw new Exception("Size of NavMesh3D leaf must be non negative.");
            }
            if (!(size.x > 0.0f && size.y > 0.0f && size.z > 0.0f)) {
                throw new Exception("Size of NavMesh3D must be non negative.");
            }
            _size = size;
            _origin = origin;
            _leafSize = leafSize;
            _graphSize[0] = (int)Mathf.Floor(_size.x / _leafSize);
            _graphSize[1] = (int)Mathf.Floor(_size.y / _leafSize);
            _graphSize[2] = (int)Mathf.Floor(_size.z / _leafSize);
            var raycastHitResults = new RaycastHit[100];
            _leafs = new Leaf[_graphSize[0] * _graphSize[1] * _graphSize[2]];
            var leafHalfExtents = new Vector3(_leafSize / 2.0f, _leafSize / 2.0f, _leafSize / 2.0f);
            for (int i = 0; i < _size.x / _leafSize; i++) {
                for (int j = 0; j < _size.y / _leafSize; j++) {
                    for (int k = 0; k < _size.z / _leafSize; k++) {
                        var leaf = new Leaf();
                        leaf.Index = i * _graphSize[1] * _graphSize[2] + j * _graphSize[2] + k;
                        _leafs[leaf.Index] = leaf;
                        leaf.Center = new Vector3(_origin.x + i * _leafSize + _leafSize * 0.5f, _origin.y + j * _leafSize + _leafSize * 0.5f, _origin.z + k * _leafSize + _leafSize * 0.5f);
                        leaf.StaticBlocked = Physics.BoxCastNonAlloc(leaf.Center, leafHalfExtents, Vector3.forward, raycastHitResults, Quaternion.identity, _leafSize / 2.0f) > 0;

                        if ((leaf.Neighbors[0] = GetLeaf(i, j, k - 1)) != null) {
                            leaf.Neighbors[0].Neighbors[4] = leaf;
                        }
                        if ((leaf.Neighbors[1] = GetLeaf(i - 1, j, k - 1)) != null) {
                            leaf.Neighbors[1].Neighbors[5] = leaf;
                        }
                        if ((leaf.Neighbors[2] = GetLeaf(i - 1, j, k)) != null) {
                            leaf.Neighbors[2].Neighbors[6] = leaf;
                        }
                        if ((leaf.Neighbors[3] = GetLeaf(i - 1, j, k + 1)) != null) {
                            leaf.Neighbors[3].Neighbors[7] = leaf;
                        }
                        if ((leaf.Neighbors[8] = GetLeaf(i, j - 1, k)) != null) {
                            leaf.Neighbors[8].Neighbors[17] = leaf;
                        }
                        if ((leaf.Neighbors[9] = GetLeaf(i, j - 1, k - 1)) != null) {
                            leaf.Neighbors[9].Neighbors[22] = leaf;
                        }
                        if ((leaf.Neighbors[10] = GetLeaf(i - 1, j - 1, k - 1)) != null) {
                            leaf.Neighbors[10].Neighbors[23] = leaf;
                        }
                        if ((leaf.Neighbors[11] = GetLeaf(i - 1, j - 1, k)) != null) {
                            leaf.Neighbors[11].Neighbors[24] = leaf;
                        }
                        if ((leaf.Neighbors[12] = GetLeaf(i - 1, j - 1, k + 1)) != null) {
                            leaf.Neighbors[12].Neighbors[25] = leaf;
                        }
                        if ((leaf.Neighbors[18] = GetLeaf(i, j + 1, k - 1)) != null) {
                            leaf.Neighbors[18].Neighbors[13] = leaf;
                        }
                        if ((leaf.Neighbors[19] = GetLeaf(i - 1, j + 1, k - 1)) != null) {
                            leaf.Neighbors[19].Neighbors[14] = leaf;
                        }
                        if ((leaf.Neighbors[20] = GetLeaf(i - 1, j + 1, k)) != null) {
                            leaf.Neighbors[20].Neighbors[15] = leaf;
                        }
                        if ((leaf.Neighbors[21] = GetLeaf(i - 1, j + 1, k + 1)) != null) {
                            leaf.Neighbors[21].Neighbors[16] = leaf;
                        }
                    }
                }
            }
        }

        /// <summary> 
        /// Create a navmesh3d from current scene environment
        /// </summary>
        /// <returns>int value</returns>
        public bool Load(byte[] data) {
            if (data.Length < 28) {
                return false;
            }
            _size = new Vector3();
            _origin = new Vector3();
            _size.x = BitConverter.ToSingle(data, 0);
            _size.y = BitConverter.ToSingle(data, 4);
            _size.z = BitConverter.ToSingle(data, 8);
            _origin.x = BitConverter.ToSingle(data, 12);
            _origin.y = BitConverter.ToSingle(data, 16);
            _origin.z = BitConverter.ToSingle(data, 20);
            _leafSize = BitConverter.ToSingle(data, 24);
            if (_leafSize <= 0.0f) {
                throw new Exception("Size of NavMesh3D leaf must be non negative.");
            }
            if (!(_size.x > 0.0f && _size.y > 0.0f && _size.z > 0.0f)) {
                throw new Exception("Size of NavMesh3D must be non negative.");
            }

            return true;
        }

        public Leaf GetLeaf(int x, int y, int z) {
            if (x < 0 || y < 0 || z < 0 || x >= _graphSize[0] || y >= _graphSize[1] || z >= _graphSize[2]) {
                return null;
            }
            var index = x * _graphSize[1] * _graphSize[2] + y * _graphSize[2] + z;
            if (index < 0 || index >= _leafs.Length) {
                return null;
            }
            return _leafs[index];
        }

        public Leaf GetLeaf(Vector3 point) {
            int x = (int)Mathf.Round((point.x - _origin.x) / _leafSize);
            int y = (int)Mathf.Round((point.y - _origin.y) / _leafSize);
            int z = (int)Mathf.Round((point.z - _origin.z) / _leafSize);
            return GetLeaf(x, y, z);
        }

        public bool GetPath(Vector3 origin, Vector3 destination, List<Vector3> corners, out int cornersCount) {
            var originLeaf = GetLeaf(origin);
            var destinationLeaf = GetLeaf(destination);

            if (originLeaf == null || destinationLeaf == null) {
                cornersCount = 0;
                return false;
            }

            if (originLeaf == destinationLeaf) {
                if (corners.Count < 1) {
                    corners.Add(originLeaf.Center);
                } else {
                    corners[0] = originLeaf.Center;
                }
                cornersCount = 1;
                return true;
            }

            if (destinationLeaf.StaticBlocked) {
                cornersCount = 0;
                return false;
            }

            // Search

            cornersCount = 0;
            while (FillLeaf(originLeaf, destinationLeaf, _searchingList, cornersCount)) {
                cornersCount++;
                if (cornersCount > _maxCorners) {
                    throw new Exception("NavMesh3D. Corners count overflow");
                }
            }

            if (cornersCount == 0) {
                return false;
            }
            
            // Optimize

            var n = 0;
            var maxStep = 0;
            for (int i = 0; i < cornersCount; i++) {
                var node = _searchingList[i];
                node.Steps = -1;
                node.NeighborsCount = 0;
            }
            for (int i = 0; i < cornersCount; i++) {
                var nodeA = _searchingList[i];
                for (int j = 0; j < cornersCount; j++) {
                    var nodeB = _searchingList[j];
                    if (nodeA != nodeB && !nodeA.InNeighbors(nodeB) && nodeA.Leaf.InNeighbors(nodeB.Leaf)) {
                        nodeA.Neighbors[nodeA.NeighborsCount] = nodeB;
                        nodeB.Neighbors[nodeB.NeighborsCount] = nodeA;
                        nodeA.NeighborsCount++;
                        nodeB.NeighborsCount++;
                    }
                }
            }

            var currentStep = 0;
            var change = true;
            var lastNode = _searchingList[cornersCount - 1];
            lastNode.Steps = currentStep;
            while (change) {
                currentStep++;
                for (var i = 0; i < cornersCount; i++) {
                    _searchingList[i].Checked = false;
                }
                change = SetNodeStep(lastNode, currentStep, false);
            }
            maxStep = 0;
            for (int i = 0; i < cornersCount; i++) {
                var node = _searchingList[i];
                if (node.Steps > maxStep) {
                    maxStep = node.Steps + 1;
                }
            }

            var currentNode = _searchingList[0];
            while (currentNode != null) {
                if (n >= _resultList.Count) {
                    _resultList.Add(currentNode.Leaf);
                } else {
                    _resultList[n] = currentNode.Leaf;
                }
                n++;
                var index = -1;
                var minStep = currentNode.Steps;
                for (int i = 0; i < currentNode.NeighborsCount; i++) {
                    var neighborNode = currentNode.Neighbors[i];
                    if (neighborNode.Steps < minStep) {
                        minStep = neighborNode.Steps;
                        index = i;
                    }
                }
                if (index == -1) {
                    currentNode = null;
                } else {
                    currentNode = currentNode.Neighbors[index];
                    if (currentNode.Leaf == destinationLeaf) {
                        currentNode = null;
                    }
                }
            }
            cornersCount = n;

            // Transfer

            for (int i = 0; i < cornersCount; i++) {
                if (i >= corners.Count) {
                    corners.Add(_resultList[cornersCount - i - 1].Center);
                } else {
                    corners[i] = _resultList[cornersCount - i - 1].Center;
                }
            }

            return true;
        }

        public bool FillLeaf(Leaf origin, Leaf target, List<LeafNode> path, int length) {
            if (origin != null) {
                if (origin == target) {
                    InsertLeafNodeInList(origin, path, length, 0.0f);
                    return false;
                }
                if (length == 0) {
                    InsertLeafNodeInList(origin, path, length, GetNodeWeight(origin, origin, target));
                    return true;
                }
                var index = -1;
                LeafNode lastNode = path[0];
                if (!lastNode.Closed) {
                    index = 0;
                }
                for (var i = 1; i < length; i++) {
                    var leafNode = path[i];
                    if (!leafNode.Closed && !leafNode.Leaf.StaticBlocked && !leafNode.Leaf.DynamicBlocked && (leafNode.Weight < lastNode.Weight || lastNode.Closed)) {
                        var opened = false;
                        for (var j = 0; j < leafNode.Leaf.Neighbors.Length; j++) {
                            var neighborLeaf = leafNode.Leaf.Neighbors[j];
                            if (neighborLeaf != null) {
                                if (!neighborLeaf.StaticBlocked && !neighborLeaf.DynamicBlocked && !IsLeafInPath(neighborLeaf, path, length)) {
                                    opened = true;
                                    j = leafNode.Leaf.Neighbors.Length;
                                }
                            }
                        }
                        if (opened) {
                            index = i;
                            lastNode = path[i];
                        } else {
                            path[i].Closed = true;
                        }
                    }
                }
                if (index != -1) {
                    var angle = 0.0f;
                    var nextLeaf = lastNode.Leaf.Neighbors[0];
                    var dir = target.Center - lastNode.Leaf.Center;
                    for (int i = 1; i < lastNode.Leaf.Neighbors.Length; i++) {
                        var leaf = lastNode.Leaf.Neighbors[i];
                        if (leaf != null) {
                            if (leaf == target) {
                                InsertLeafNodeInList(origin, path, length, 0.0f);
                                return false;
                            }
                            if (!leaf.StaticBlocked && !leaf.DynamicBlocked) {
                                if (nextLeaf == null) {
                                    nextLeaf = leaf;
                                } else {
                                    var angleA = Vector3.Angle(nextLeaf.Center - lastNode.Leaf.Center, dir);
                                    var angleB = Vector3.Angle(leaf.Center - lastNode.Leaf.Center, dir);
                                    var weightA = (nextLeaf.Center - target.Center).magnitude + angleA / 90.0f * _leafSize;
                                    var weightB = (leaf.Center - target.Center).magnitude + angleB / 90.0f * _leafSize;
                                    if ((weightB < weightA || IsLeafInPath(nextLeaf, path, length)) && !IsLeafInPath(leaf, path, length)) {
                                        nextLeaf = leaf;
                                        angle = angleB;
                                    }
                                }
                            }
                        }
                    }
                    if (nextLeaf != null && !IsLeafInPath(nextLeaf, path, length)) {
                        InsertLeafNodeInList(nextLeaf, path, length, GetNodeWeight(nextLeaf, lastNode.Leaf, target));
                        return true;
                    }
                }
            }
            return false;
        }

        public void InsertLeafNodeInList(Leaf leaf, List<LeafNode> path, int index, float weight) {
            LeafNode leafNode;
            if (index >= path.Count) {
                leafNode = new LeafNode();
                path.Add(leafNode);
            } else {
                leafNode = path[index];
            }
            leafNode.Leaf = leaf;
            leafNode.Weight = weight;
            leafNode.Closed = false;
        }

        public int FindLeaf(Leaf origin, Leaf target, List<LeafNode> path, int depth) {
            if (depth >= path.Count) {
                var leafNode = new LeafNode();
                leafNode.Leaf = origin;
                leafNode.Weight = depth;
                path.Add(leafNode);
            } else {
                path[depth].Leaf = origin;
            }
            for (int i = 0; i < origin.Neighbors.Length; i++) {
                if (origin.Neighbors[i] == target) {
                    return depth;
                }
            }

            // Fill

            var totalIndexes = 0;
            for (int i = 0; i < origin.Neighbors.Length; i++) {
                var leaf = origin.Neighbors[i];
                if (leaf != null && !leaf.StaticBlocked && !leaf.DynamicBlocked) {
                    var inPath = false;
                    for (int j = 0; j < depth; j++) {
                        if (path[j].Leaf == leaf) {
                            inPath = true;
                            j = depth;
                        }
                    }
                    if (!inPath) {
                        _indexes[totalIndexes] = i;
                        totalIndexes++;
                    }
                }
            }

            //Sort

            var dir = target.Center - origin.Center;
            for (int i = 1; i < totalIndexes; i++) {
                for (int j = totalIndexes - 1; j >= i; j--) {
                    var leafA = origin.Neighbors[_indexes[j - 1]];
                    var leafB = origin.Neighbors[_indexes[j]];
                    if ((leafB.Center - origin.Center - dir).sqrMagnitude < (leafA.Center - origin.Center - dir).sqrMagnitude) {
                        var idx = _indexes[j - 1];
                        _indexes[j - 1] = _indexes[j];
                        _indexes[j] = idx;
                    }
                }
            }

            // Search

            for (int i = 0; i < totalIndexes; i++) {
                var leaf = origin.Neighbors[_indexes[i]];
                return FindLeaf(leaf, target, path, depth + 1);
            }

            return -1;
        }

        public bool IsLeafInPath(Leaf leaf, List<LeafNode> path, int length) {
            return FindLeafInPath(leaf, path, length) != null;
        }

        public LeafNode FindLeafInPath(Leaf leaf, List<LeafNode> path, int length) {
            for (var i = 0; i < length; i++) {
                if (leaf == path[i].Leaf) {
                    return path[i];
                }
            }
            return null;
        }

        public float GetNodeWeight(Leaf leaf, Leaf origin, Leaf destination) {
            var weight = (destination.Center - leaf.Center).magnitude;
            if (leaf != origin) {
                var angle = Vector3.Angle(leaf.Center - origin.Center, destination.Center - origin.Center);
                weight += angle / 90.0f * _leafSize;
            }
            return weight;
        }

        public bool SetNodeStep(LeafNode root, int step, bool final) {
            var change = false;
            for (var i = 0; i < root.NeighborsCount; i++) {
                var neighbor = root.Neighbors[i];
                if (neighbor.Steps == -1) {
                    change = true;
                    neighbor.Steps = step;
                    neighbor.Checked = true;
                } else if (neighbor.Steps > root.Steps + 1) {
                    if (!change) {
                        change = true;
                    }
                    neighbor.Steps = root.Steps + 1;
                    neighbor.Checked = true;
                } else {
                }
            }
            if (change) {
                return true;
            } else if (final) {
                return false;
            }
            for (var i = 0; i < root.NeighborsCount; i++) {
                var neighbor = root.Neighbors[i];
                if (neighbor.Steps > root.Steps && !neighbor.Checked) {
                    neighbor.Checked = true;
                    if (SetNodeStep(neighbor, step, final) && !change) {
                        final = true;
                        change = true;
                    }
                }
            }
            return change;
        }

    }

}