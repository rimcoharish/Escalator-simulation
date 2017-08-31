/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

#include "cs296_base.hpp"
#include "render.hpp"
#include <cstring>

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include "dominos.hpp"

using namespace std;

/** The cs296 namespace contains the function definition for the default constructor of the class dominos_t,
 *  which is declared in dominos.hpp. In addition, it also contains a variable sim, of type sim_t,
 *  which is initialized using new to create the scene.
 */

namespace cs296
{
	/** The default constructor of the class dominos_t declares and defines the various objects present in the scene.
	 *
	 *  Our escalator simulation consists of the following objects:
	 */

	dominos_t::dominos_t()
	{
		/**   -  The ground
		 *
		 *       The ground is implemented as a horizontal edge, stretching from (-90, 0) to (90, 0),
		 *       across the bottom of the scene.
		 */

		b2Body* ground;  
		{ 
			b2EdgeShape shape; 
			shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
			b2BodyDef bd; 
			ground = m_world->CreateBody(&bd); 
			ground->CreateFixture(&shape, 0.0f);
		}

		/**   -  The motor
		 *
		 *       The motor is a self-driven component which drives the step chain gear.
		 *       It is implemented as a circular object, and, like all other stationary escalator components,
		 *       is affixed to the background of the scene by a revolute joint.
		 */

		b2Body* motor;
		{
			b2CircleShape circle;
			circle.m_radius = 1.5;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 50.0f;
			ballfd.friction = 1.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(-20.0f, 30.0f);
			motor = m_world->CreateBody(&ballbd);
			motor->CreateFixture(&ballfd);

			b2PolygonShape shape2;
			shape2.SetAsBox(0.2f, 2.0f);
			b2BodyDef bd2;
			bd2.position.Set(-25.0f, 30.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = motor;
			jointDef.bodyB = body2;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			jointDef.motorSpeed = -6;
			jointDef.maxMotorTorque = 10000000;
			jointDef.enableMotor = true;
			jointDef.enableLimit = false;

			m_world->CreateJoint(&jointDef);
		}

		/**   -  The first step chain gear
		 *
		 *       This first step chain gear is connected to the motor by a belt, and is driven by it.
		 *       The gears are all implemented as circular objects.
		 */

		b2Body* chain_gear;
		{
			b2CircleShape circle;
			circle.m_radius = 3.0;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 50.0f;
			ballfd.friction = 1.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(-10.0f, 30.0f);
			chain_gear = m_world->CreateBody(&ballbd);
			chain_gear->CreateFixture(&ballfd);

			b2PolygonShape shape2;
			shape2.SetAsBox(0.2f, 2.0f);
			b2BodyDef bd2;
			bd2.position.Set(-15.0f, 30.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = chain_gear;
			jointDef.bodyB = body2;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			jointDef.motorSpeed = -2.5;
			jointDef.maxMotorTorque = 10000000;
			jointDef.enableMotor = true;
			jointDef.enableLimit = false;

			m_world->CreateJoint(&jointDef);
		}

		/**   -  The second step chain gear
		 *
		 *       This second step chain gear is driven automatically by the motion of the first.
		 */

		b2Body* gear;
		{
			b2CircleShape circle;
			circle.m_radius = 3.0;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 50.0f;
			ballfd.friction = 1.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(10.0f, 10.0f);
			gear = m_world->CreateBody(&ballbd);
			gear->CreateFixture(&ballfd);

			b2PolygonShape shape2;
			shape2.SetAsBox(0.2f, 2.0f);
			b2BodyDef bd2;
			bd2.position.Set(10.0f, 5.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = gear;
			jointDef.bodyB = body2;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;

			m_world->CreateJoint(&jointDef);
		}

		//Support for the conveyor belt
		for (int i=0; i<4; i++){
			{
				b2CircleShape circle;
				circle.m_radius = 1;
				
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.filter.categoryBits = 0x0002;
				ballfd.density = 50.0f;
				ballfd.friction = 1.0f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(10.0f, 10.0f);
				gear = m_world->CreateBody(&ballbd);
				gear->CreateFixture(&ballfd);

				b2PolygonShape shape2;
				shape2.SetAsBox(0.2f, 2.0f);
				b2BodyDef bd2;
				bd2.position.Set(-15.0+(5*(i+1))+(3-circle.m_radius)/1.414, 30.0-(5*(i+1))+(3-circle.m_radius)/1.414);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = gear;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;

				m_world->CreateJoint(&jointDef);
			}

			{
				b2CircleShape circle;
				circle.m_radius = 1;
				
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.filter.categoryBits = 0x0002;
				ballfd.density = 50.0f;
				ballfd.friction = 1.0f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(10.0f, 10.0f);
				gear = m_world->CreateBody(&ballbd);
				gear->CreateFixture(&ballfd);

				b2PolygonShape shape2;
				shape2.SetAsBox(0.2f, 2.0f);
				b2BodyDef bd2;
				bd2.position.Set(-15.0+(5*(i+1))+(3-circle.m_radius)/1.414-(3*1.414), 30.0-(5*(i+1))+(3-circle.m_radius)/1.414-(3*1.414));
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = gear;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;

				m_world->CreateJoint(&jointDef);
			}
		}

		/**   -  The belt
		 *
		 *       This is the belt connecting the motor to the first step chain gear, and thus driving it.
		 *       The belt, like the step chain and the handrail, is implemented as consisting of
		 *       several small rectangular pieces attached to each other by revolute joints.
		 *       It is constructed in four sections, two going over the top and bottom,
		 *       and two going over the sides, which are then affixed to each other appropriately.
		 */

		{
			int up_pieces = 14, down_pieces = 14, left_pieces = 2, right_pieces = 3;
			int up_x = -26, up_y = 32;
			int down_x = -26, down_y = 28;
			float32 x = 0.5, y = 0.1;

			b2PolygonShape shape_h;
			shape_h.SetAsBox(x, y);

			b2PolygonShape shape_v;
			shape_v.SetAsBox(y, x);

			b2FixtureDef fd_h;
			fd_h.filter.groupIndex = -8;
			fd_h.shape = &shape_h;
			fd_h.density = 50.0f;
			fd_h.friction = 1.0f;
			fd_h.restitution = 0.0f;

			b2FixtureDef fd_v;
			fd_v.filter.groupIndex = -8;
			fd_v.shape = &shape_v;
			fd_v.density = 50.0f;
			fd_v.friction = 1.0f;
			fd_v.restitution = 0.0f;

			b2RevoluteJointDef jointDef;
			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(x,0);

			b2RevoluteJointDef jointDef1;
			jointDef1.localAnchorA.Set(0,-x);
			jointDef1.localAnchorB.Set(0,x);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(up_x, up_y);

			b2Body* body_up[up_pieces];
			body_up[0] = m_world->CreateBody(&bd);
			body_up[0]->CreateFixture(&fd_h);

			for(int i = 1; i<up_pieces; i++){
				bd.position.Set(up_x + 2 * x * i, up_y);
				body_up[i] = m_world->CreateBody(&bd);
				body_up[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_up[i];
				jointDef.bodyB = body_up[i-1];
				m_world->CreateJoint(&jointDef);
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_down[down_pieces];
			body_down[0] = m_world->CreateBody(&bd);
			body_down[0]->CreateFixture(&fd_h);

			for(int i = 1; i<down_pieces; i++){
				bd.position.Set(down_x + 2 * x * i, down_y);
				body_down[i] = m_world->CreateBody(&bd);
				body_down[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_down[i];
				jointDef.bodyB = body_down[i-1];
				m_world->CreateJoint(&jointDef);
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_left[left_pieces];
			body_left[0] = m_world->CreateBody(&bd);
			body_left[0]->CreateFixture(&fd_v);

			for(int i = 1; i<left_pieces; i++){
				bd.position.Set(down_x, down_y + 2 * x * i);
				body_left[i] = m_world->CreateBody(&bd);
				body_left[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_left[i];
				jointDef1.bodyB = body_left[i-1];
				m_world->CreateJoint(&jointDef1);
			};

			bd.position.Set(down_x+down_pieces, down_y);
			b2Body* body_right[right_pieces];
			body_right[0] = m_world->CreateBody(&bd);
			body_right[0]->CreateFixture(&fd_v);

			for(int i = 1; i<right_pieces; i++){
				bd.position.Set(down_x+down_pieces, down_y + 2 * x * i);
				body_right[i] = m_world->CreateBody(&bd);
				body_right[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_right[i];
				jointDef1.bodyB = body_right[i-1];
				m_world->CreateJoint(&jointDef1);
			};

			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[0];
			jointDef.bodyB = body_left[left_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[0];
			jointDef.bodyB = body_left[0];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[up_pieces-1];
			jointDef.bodyB = body_right[right_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[down_pieces-1];
			jointDef.bodyB = body_right[0];
			m_world->CreateJoint(&jointDef);
		}

		/**   -  The step chain
		 *
		 *       The step chain drives the steps up the escalator.
		 *       In our model of the escalator, the steps are implemented simply as
		 *       massless square objects attached to the pieces of the step chain.
		 */

		{
			int up_pieces = 19, down_pieces = 19, left_pieces = 2, right_pieces = 2;
			int up_x = -15 , up_y = 30 + (3*1.414);
			int down_x = -15 - (3/1.414), down_y = 30, down_x1 = 13, down_y1 = 5;
			float32 x = 1, y = 0.1;
			float32 l = 1.5;

			b2PolygonShape shape_step;
			shape_step.SetAsBox(l, l);

			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 50.0f;
			fd.friction = 10.0f;
			fd.restitution = 0.0f;

			b2PolygonShape shape_h;
			shape_h.SetAsBox(x, y);

			b2PolygonShape shape_v;
			shape_v.SetAsBox(y, x);

			b2FixtureDef fd_step;
			fd_step.filter.groupIndex = -8;
			fd_step.filter.categoryBits = 0x0002;
			fd_step.shape = &shape_step;
			fd_step.density = 0.0f;
			fd_step.friction = 1.0f;
			fd_step.restitution = 0.0f;

			b2FixtureDef fd_h;
			fd_h.filter.groupIndex = -8;
			fd_h.shape = &shape_h;
			fd_h.density = 50.0f;
			fd_h.friction = 1.0f;
			fd_h.restitution = 0.0f;

			b2FixtureDef fd_v;
			fd_v.filter.groupIndex = -8;
			fd_v.shape = &shape_v;
			fd_v.density = 50.0f;
			fd_v.friction = 1.0f;
			fd_v.restitution = 0.0f;

			b2RevoluteJointDef jointDef_step;
			jointDef_step.localAnchorA.Set(-l,-l);
			jointDef_step.localAnchorB.Set(0,0);
			jointDef_step.motorSpeed = 0;

			b2RevoluteJointDef jointDef;
			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(x,0);

			b2RevoluteJointDef jointDef1;
			jointDef1.localAnchorA.Set(0,-x);
			jointDef1.localAnchorB.Set(0,x);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.angle = -45;

			bd.position.Set(up_x, up_y);
			b2Body* body_up[up_pieces];
			body_up[0] = m_world->CreateBody(&bd);
			body_up[0]->CreateFixture(&fd_h);

			for(int i = 1; i<up_pieces; i++)
			{
				bd.position.Set(up_x + 2 * x * i / 1.414, up_y - 2 * x * i / 1.414);
				body_up[i] = m_world->CreateBody(&bd);
				body_up[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_up[i];
				jointDef.bodyB = body_up[i-1];
				m_world->CreateJoint(&jointDef);

				if (i%2 == 1)
				{
					b2BodyDef bd_step;
					bd_step.fixedRotation = true;
					bd_step.type = b2_dynamicBody;
					bd.position.Set(up_x + l + 2 * x * i / 1.414, up_y + l + - 2 * x * i / 1.414);
					b2Body* step;
					step = m_world->CreateBody(&bd_step);
					step->CreateFixture(&fd_step);
					jointDef_step.bodyA = step;
					jointDef_step.bodyB = body_up[i-1];
					m_world->CreateJoint(&jointDef_step);
				}

				if (i == up_pieces-1)
				{
					b2BodyDef bd_step;
					bd_step.fixedRotation = true;
					bd_step.type = b2_dynamicBody;
					bd.position.Set(up_x + l + 2 * x * i / 1.414, up_y + l + - 2 * x * i / 1.414);
					b2Body* step;
					step = m_world->CreateBody(&bd_step);
					step->CreateFixture(&fd_step);
					jointDef_step.bodyA = step;
					jointDef_step.bodyB = body_up[i];
					m_world->CreateJoint(&jointDef_step);
				}
				
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_down[down_pieces];
			body_down[0] = m_world->CreateBody(&bd);
			body_down[0]->CreateFixture(&fd_h);

			for(int i = 1; i<down_pieces; i++)
			{
				bd.position.Set(down_x + 2 * x * i / 1.414, down_y - 2 * x * i / 1.414);
				body_down[i] = m_world->CreateBody(&bd);
				body_down[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_down[i];
				jointDef.bodyB = body_down[i-1];
				m_world->CreateJoint(&jointDef);

				if (i%2 == 0)
				{
					b2BodyDef bd_step;
					bd_step.fixedRotation = true;
					bd_step.type = b2_dynamicBody;
					bd.position.Set(up_x + l + 2 * x * i / 1.414, up_y + l + - 2 * x * i / 1.414);
					b2Body* step;
					step = m_world->CreateBody(&bd_step);
					step->CreateFixture(&fd_step);
					jointDef_step.bodyA = step;
					jointDef_step.bodyB = body_down[i-1];
					m_world->CreateJoint(&jointDef_step);
				}
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_left[left_pieces];
			body_left[0] = m_world->CreateBody(&bd);
			body_left[0]->CreateFixture(&fd_v);

			for(int i = 1; i<left_pieces; i++)
			{
				bd.position.Set(down_x - 2 * x * i / 1.414, down_y + 2 * x * i / 1.414);
				body_left[i] = m_world->CreateBody(&bd);
				body_left[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_left[i];
				jointDef1.bodyB = body_left[i-1];
				m_world->CreateJoint(&jointDef1);

				if (i%2 == 1)
				{
					b2BodyDef bd_step;
					bd_step.fixedRotation = true;
					bd_step.type = b2_dynamicBody;
					bd.position.Set(up_x + l + 2 * x * i / 1.414, up_y + l + - 2 * x * i / 1.414);
					b2Body* step;
					step = m_world->CreateBody(&bd_step);
					step->CreateFixture(&fd_step);
					jointDef_step.bodyA = step;
					jointDef_step.bodyB = body_left[i-1];
					m_world->CreateJoint(&jointDef_step);
				}
			};

			bd.position.Set(down_x1, down_y1);
			b2Body* body_right[right_pieces];
			body_right[0] = m_world->CreateBody(&bd);
			body_right[0]->CreateFixture(&fd_v);

			for(int i = 1; i<right_pieces; i++)
			{
				bd.position.Set(down_x1, down_y1 + 2 * x * i / 1.414);
				body_right[i] = m_world->CreateBody(&bd);
				body_right[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_right[i];
				jointDef1.bodyB = body_right[i-1];
				m_world->CreateJoint(&jointDef1);

				if (i%2 == 1)
				{
					b2BodyDef bd_step;
					bd_step.fixedRotation = true;
					bd_step.type = b2_dynamicBody;
					bd.position.Set(up_x + l + 2 * x * i / 1.414, up_y + l + - 2 * x * i / 1.414);
					b2Body* step;
					step = m_world->CreateBody(&bd_step);
					step->CreateFixture(&fd_step);
					jointDef_step.bodyA = step;
					jointDef_step.bodyB = body_right[i-1];
					m_world->CreateJoint(&jointDef_step);
				}
			};
			
			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[0];
			jointDef.bodyB = body_left[left_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[0];
			jointDef.bodyB = body_left[0];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[up_pieces-1];
			jointDef.bodyB = body_right[right_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[down_pieces-1];
			jointDef.bodyB = body_right[0];
			m_world->CreateJoint(&jointDef);
		}

		/**   -  The motorized handrail gear
		 *
		 *       The first handrail gear is motorized, and, thus, self-driven.
		 */

		b2Body* hr_motor;
		{
			b2CircleShape circle;
			circle.m_radius = 2.0;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 50.0f;
			ballfd.friction = 1.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(-10.0f, 30.0f);
			hr_motor = m_world->CreateBody(&ballbd);
			hr_motor->CreateFixture(&ballfd);

			b2PolygonShape shape2;
			shape2.SetAsBox(0.2f, 2.0f);
			b2BodyDef bd2;
			bd2.position.Set(-14.5f, 45.5f);
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = hr_motor;
			jointDef.bodyB = body2;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			jointDef.motorSpeed = -2.5;
			jointDef.maxMotorTorque = 10000000;
			jointDef.enableMotor = true;
			jointDef.enableLimit = false;

			m_world->CreateJoint(&jointDef);
		}

		/**   -  The second handrail gear
		 *
		 *       This second handrail gear is driven automatically by the motion of the first.
		 */

		b2Body* hr_chain_gear;
		{
			b2CircleShape circle;
			circle.m_radius = 2.0;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 50.0f;
			ballfd.friction = 1.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(10.0f, 10.0f);
			hr_chain_gear = m_world->CreateBody(&ballbd);
			hr_chain_gear->CreateFixture(&ballfd);

			b2PolygonShape shape2;
			shape2.SetAsBox(0.2f, 2.0f);
			b2BodyDef bd2;
			bd2.position.Set(14.5f, 16.5f);
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = hr_chain_gear;
			jointDef.bodyB = body2;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;

			m_world->CreateJoint(&jointDef);
		}

		//Support for the Hand Rail
		for (int i=0; i<5; i++){
			{
				b2CircleShape circle;
				circle.m_radius = 1;
				
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.filter.categoryBits = 0x0002;
				ballfd.density = 50.0f;
				ballfd.friction = 1.0f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(10.0f, 10.0f);
				gear = m_world->CreateBody(&ballbd);
				gear->CreateFixture(&ballfd);

				b2PolygonShape shape2;
				shape2.SetAsBox(0.2f, 2.0f);
				b2BodyDef bd2;
				bd2.position.Set(-14.5+(5*(i+1))+(3-circle.m_radius)/1.414, 45.5-(5*(i+1))+(3-circle.m_radius)/1.414);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = gear;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;

				m_world->CreateJoint(&jointDef);
			}

			{
				b2CircleShape circle;
				circle.m_radius = 1;
				
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.filter.categoryBits = 0x0002;
				ballfd.density = 50.0f;
				ballfd.friction = 1.0f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(10.0f, 10.0f);
				gear = m_world->CreateBody(&ballbd);
				gear->CreateFixture(&ballfd);

				b2PolygonShape shape2;
				shape2.SetAsBox(0.2f, 2.0f);
				b2BodyDef bd2;
				bd2.position.Set(-14.5+(5*(i+1))+(3-circle.m_radius)/1.414-(2*1.414), 45.5-(5*(i+1))+(3-circle.m_radius)/1.414-(2*1.414));
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = gear;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;

				m_world->CreateJoint(&jointDef);
			}
		}

		/**   -  The handrail
		 *
		 *       The handrail is wrapped around the handrail gears to begin with
		 *       and loops around them as the simulation progresses.
		 */

		{
			int up_pieces = 22, down_pieces = 22, left_pieces = 1, right_pieces = 1;
			int up_x = -14.5 , up_y = 45.5 + (3*1.414);
			int down_x = -14.5 - (3/1.414), down_y = 45.5, down_x1 = 14.5, down_y1 = 16.5;
			float32 x = 1, y = 0.1;

			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 50.0f;
			fd.friction = 10.0f;
			fd.restitution = 0.0f;

			b2PolygonShape shape_h;
			shape_h.SetAsBox(x, y);

			b2PolygonShape shape_v;
			shape_v.SetAsBox(y, x);

			b2FixtureDef fd_h;
			fd_h.filter.groupIndex = -8;
			fd_h.shape = &shape_h;
			fd_h.density = 50.0f;
			fd_h.friction = 1.0f;
			fd_h.restitution = 0.0f;

			b2FixtureDef fd_v;
			fd_v.filter.groupIndex = -8;
			fd_v.shape = &shape_v;
			fd_v.density = 50.0f;
			fd_v.friction = 1.0f;
			fd_v.restitution = 0.0f;

			b2RevoluteJointDef jointDef;
			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(x,0);

			b2RevoluteJointDef jointDef1;
			jointDef1.localAnchorA.Set(0,-x);
			jointDef1.localAnchorB.Set(0,x);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.angle = -45;

			bd.position.Set(up_x, up_y);
			b2Body* body_up[up_pieces];
			body_up[0] = m_world->CreateBody(&bd);
			body_up[0]->CreateFixture(&fd_h);

			for(int i = 1; i<up_pieces; i++)
			{
				bd.position.Set(up_x + 2 * x * i / 1.414, up_y - 2 * x * i / 1.414);
				body_up[i] = m_world->CreateBody(&bd);
				body_up[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_up[i];
				jointDef.bodyB = body_up[i-1];
				m_world->CreateJoint(&jointDef);
				
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_down[down_pieces];
			body_down[0] = m_world->CreateBody(&bd);
			body_down[0]->CreateFixture(&fd_h);

			for(int i = 1; i<down_pieces; i++)
			{
				bd.position.Set(down_x + 2 * x * i / 1.414, down_y - 2 * x * i / 1.414);
				body_down[i] = m_world->CreateBody(&bd);
				body_down[i]->CreateFixture(&fd_h);
				jointDef.bodyA = body_down[i];
				jointDef.bodyB = body_down[i-1];
				m_world->CreateJoint(&jointDef);
			}

			bd.position.Set(down_x, down_y);
			b2Body* body_left[left_pieces];
			body_left[0] = m_world->CreateBody(&bd);
			body_left[0]->CreateFixture(&fd_v);

			for(int i = 1; i<left_pieces; i++)
			{
				bd.position.Set(down_x - 2 * x * i / 1.414, down_y + 2 * x * i / 1.414);
				body_left[i] = m_world->CreateBody(&bd);
				body_left[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_left[i];
				jointDef1.bodyB = body_left[i-1];
				m_world->CreateJoint(&jointDef1);
			};

			bd.position.Set(down_x1, down_y1);
			b2Body* body_right[right_pieces];
			body_right[0] = m_world->CreateBody(&bd);
			body_right[0]->CreateFixture(&fd_v);

			for(int i = 1; i<right_pieces; i++)
			{
				bd.position.Set(down_x1, down_y1 + 2 * x * i / 1.414);
				body_right[i] = m_world->CreateBody(&bd);
				body_right[i]->CreateFixture(&fd_v);
				jointDef1.bodyA = body_right[i];
				jointDef1.bodyB = body_right[i-1];
				m_world->CreateJoint(&jointDef1);
			};
			
			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[0];
			jointDef.bodyB = body_left[left_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(-x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[0];
			jointDef.bodyB = body_left[0];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,x);
			jointDef.bodyA = body_up[up_pieces-1];
			jointDef.bodyB = body_right[right_pieces-1];
			m_world->CreateJoint(&jointDef);

			jointDef.localAnchorA.Set(x,0);
			jointDef.localAnchorB.Set(0,-x);
			jointDef.bodyA = body_down[down_pieces-1];
			jointDef.bodyB = body_right[0];
			m_world->CreateJoint(&jointDef);
		}
	}

	/*! The sim variable, of type sim_t, initializes a new scene using new. */

	sim_t *sim = new sim_t("Escalator", dominos_t::create);
}
