namespace ChupakaberGames.NavMesh3D {

    public class LeafNode {
        public Leaf Leaf;
        public float Weight;
        public int Steps;
        public LeafNode[] Neighbors = new LeafNode[52];
        public int NeighborsCount = 0;
        public bool Closed = false;
        public bool Checked = false;

        public bool InNeighbors(LeafNode leaf) {
            if (leaf == null) {
                return false;
            }
            for (var i = 0; i < NeighborsCount; i++) {
                if (Neighbors[i] == leaf) {
                    return true;
                }
            }
            return false;
        }

        public void Break(int index) {
            for (var i = index + 1; i < NeighborsCount; i++) {
                Neighbors[i - 1] = Neighbors[i];
            }
            NeighborsCount--;
        }

        public void Break(LeafNode node) {
            for (var i = 0; i < NeighborsCount; i++) {
                if (Neighbors[i] == node) {
                    Break(i);
                    return;
                }
            }
        }
    }
}