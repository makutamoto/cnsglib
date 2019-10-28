#ifndef NODE_H
#define NODE_H

#include<time.h>

#include "./graphics.h"
#include "./vector.h"

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
	float position[3];
	float angle[3];
	float scale[3];
	float lastTransformation[4][4];
	float aabb[3][2];
	Image texture;
	Shape shape;
	Shape collisionShape;
	unsigned int collisionFlags;
	unsigned int collisionMaskActive;
	unsigned int collisionMaskPassive;
	Vector collisionTargets;
	Vector intervalEvents;
	struct _Node *parent;
	Vector children;
	int (*behaviour)(struct _Node*);
	BOOL isPhysicsEnabled;
	BOOL isInterface;
	void *data;
}	Node;

typedef struct {
	clock_t begin;
	unsigned int interval;
	void (*callback)(Node*);
} IntervalEventNode;

Node initNode(const char *id, Image image);
Node initNodeUI(const char *id, Image image, unsigned char color);
void discardNode(Node node);

void drawNode(Node *node);
void applyForce(Node *node, float force[3], int mask, int rotation);
float (*getNodeTransformation(Node node, float out[4][4]))[4];
float (*getWorldTransfomration(Node node, float out[4][4]))[4];
int testCollision(Node a, Node b);
int testCollisionPolygonPolygon(Node a, Node b, Vector *normals, Vector *points);
void addIntervalEventNode(Node *node, unsigned int milliseconds, void (*callback)(Node*));

Shape initShape(float mass);
Shape initShapePlane(float width, float height, unsigned char color);
Shape initShapePlaneInv(float width, float height, unsigned char color);
Shape initShapeBox(float width, float height, float depth, unsigned char color);
int initShapeFromObj(Shape *shape, char *filename, float mass);
void discardShape(Shape shape);

#endif
