#ifndef NODE_H
#define NODE_H

#include<time.h>

#include "./graphics.h"
#include "./vector.h"

typedef enum {
	LEFT, CENTER, RIGHT, TOP, BOTTOM
} Align;

typedef enum {
	PHYSICS_NONE, PHYSICS_2D, PHYSICS_3D
} PhysicsMode;

typedef struct {
	Vector indices;
	Vector vertices;
	Vector normals;
	Vector normalIndices;
	Vector uv;
	Vector uvIndices;
	float mass;
	float restitution;
	float staticFriction;
	float dynamicFriction;
	float rollingFriction;
	float inertia[3][3];
	float inverseInertia[3][3];
	float worldInverseInertia[3][3];
} Shape;

typedef struct _Node {
	char id[10];
	float velocity[3];
	float angVelocity[3];
	float angMomentum[3];
	float force[3];
	float torque[3];
	float previousPosition[3];
	float position[3];
	float angle[3];
	float scale[3];
	float lastTransformation[4][4];
	float aabb[3][2];
	Image texture;
	Image collisionTexture;
	Shape shape;
	Shape collisionShape;
	unsigned int collisionFlags;
	unsigned int collisionMaskActive;
	unsigned int collisionMaskPassive;
	Vector collisionTargets;
	Vector intervalEvents;
	struct _Node *parent;
	Vector children;
	int (*behaviour)(struct _Node*, float);
	PhysicsMode physicsMode;
	int isInterface;
	Align interfaceAlign[2];
	int isVisible;
	int isThrough;
	void *data;
}	Node;

typedef struct {
	Vector iterStack;
	VectorIter currentIter;
	Node *currentNode;
} NodeIter;

typedef struct {
	clock_t begin;
	unsigned int interval;
	void (*callback)(Node*);
} IntervalEventNode;

typedef struct {
	float normal[3];
	float contacts[2][3];
	float barycentric[2][3];
	float depth;
} CollisionInfoNode2Node;

Node initNode(const char *id, Image image);
Node initNodeUI(const char *id, Image image, unsigned char color);
Node initNodeSprite(const char *id, float width, float height, Image texture, Image collisionTexture);
Node initNodeText(const char *id, float px, float py, Align alignX, Align alignY, unsigned int sx, unsigned int sy, int (*behaviour)(Node*, float));
NodeIter initNodeIter(Vector *layer);
Node* nextNode(NodeIter *iter);
void addNodeChild(Node *parent, Node *child);
void discardNode(Node node);

void drawNode(Node *node, float zBuffer[], Node *replacedNode, unsigned char filter, Image *output);
void applyForce(Node *node, float force[3], int mask, int rotation);
float (*getNodeTransformation(Node node, float out[4][4]))[4];
float (*getWorldTransfomration(Node *node, float out[4][4]))[4];

CollisionInfoNode2Node initCollisionInfoNode2Node(Node *nodeA, Node *nodeB, float triangle[3][3], unsigned long normalIndex, unsigned long *uvIndex[3], float contacts[2][3], float depth);
int testCollision(Node a, Node b);
int testCollisionPolygonPolygon(Node a, Node b, Vector *infoAOut, Vector *infoBOut);
void addIntervalEventNode(Node *node, unsigned int milliseconds, void (*callback)(Node*));

Shape initShape(float mass);
Shape initShapePlane(float width, float height, unsigned char color, float mass);
Shape initShapePlaneV(float width, float height, unsigned char color);
Shape initShapePlaneInv(float width, float height, unsigned char color);
Shape initShapeBox(float width, float height, float depth, unsigned char color, float mass);
int initShapeFromObj(Shape *shape, char *filename, float mass);
float (*getShapeAABB(Shape shape, float transformation[4][4], float out[3][2]))[2];
void discardShape(Shape shape);

#endif
