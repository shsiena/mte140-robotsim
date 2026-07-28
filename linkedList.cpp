struct Node {
    Node* next = nullptr;
    Node* prev = nullptr;
    float heading = 0.0;
    float xPos = 0.0;
    float yPos = 0.0;
};

class LinkedList {
private:
    Node* root_ = nullptr;
    uint8_t numNodes = 0;

public:
    ~LinkedList() {
        Node* currentNode = root_;
        while (currentNode != nullptr) {
            Node* nextNode = currentNode->next;
            delete currentNode;
            currentNode = nextNode;
        }
        root_ = nullptr;
        numNodes = 0;
        }

    void add(float x, float y, bool vertical) {
        Node* newNode = new Node;
        if (root_ == nullptr) {
            root_ = newNode;
            numNodes++;
            return;
        }

        Node* currentNode = root_;
        while (currentNode->next != nullptr) {
            currentNode = currentNode->next;
        }

        currentNode->next = newNode;
        newNode->prev = currentNode;

        float prevX = numNodes->xPos;
        float prevY = numNodes->yPos;
        newNode->xPos = xPos + x;
        newNode->yPos = yPos + y;
        numNodes++;
    }
};