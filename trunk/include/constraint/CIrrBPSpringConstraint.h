#ifndef CIrrBP_SPRING_CONST_H_
#define CIrrBP_SPRING_CONST_H_

#include "constraint/CIrrBPConstraint.h"
#include "constraint/CIrrBP6DOFConstraint.h"

class CIrrBPSpringConstraint : public CIrrBPConstraint
{
public:
	/**Constructor.<br>
	The last 2 parameters will only works if there is a static object.<br>
	Please also note that due to internal conversions, if you place the moving body to a 0.0f position, the object will be moved toward a NEAR 0 value.<br>

	@param bodyA The first body
	@param bodyB The second body
	@param pivotInA The constraint position in A (0,0,0 by default)
	@param pivotInB The constraint position in B (0,0,0 by default)
	@param autoadapt If one body is static, and this flag is setted to false. The spring constraint will be only orthogonal
	*/
	CIrrBPSpringConstraint(CIrrBPRigidBody *bodyA, CIrrBPRigidBody *bodyB,const irr::core::vector3df & pivotInA=irr::core::vector3df(0,0,0),const irr::core::vector3df & pivotInB=irr::core::vector3df(0,0,0),bool autoadapt=true);

	irr::core::vector3df getAxis(int axis_index) const { return bullet::bulletVectorToIrrVector(m_fixedConstraint->getAxis(axis_index));}
	irr::f32 getAngle(int axis_index) const { return m_fixedConstraint->getAngle(axis_index);}

	irr::f32 getRelativePivotPosition(int axis_index) const { return m_fixedConstraint->getRelativePivotPosition(axis_index);}

	bool testAngularLimitMotor(int axis_index) { return m_fixedConstraint->testAngularLimitMotor(axis_index);}

    void	setLinearLowerLimit(const irr::core::vector3df& linearLower)
    {
    	m_fixedConstraint->setLinearLowerLimit(bullet::irrVectorToBulletVector(linearLower));
    }

    void	setLinearUpperLimit(const irr::core::vector3df& linearUpper)
    {
    	m_fixedConstraint->setLinearUpperLimit(bullet::irrVectorToBulletVector(linearUpper));
    }

    void	setAngularLowerLimit(const irr::core::vector3df& angularLower)
    {
		m_fixedConstraint->setAngularLowerLimit(bullet::irrVectorToBulletVector(angularLower));
    }

    void	setAngularUpperLimit(const irr::core::vector3df& angularUpper)
    {
		m_fixedConstraint->setAngularUpperLimit(bullet::irrVectorToBulletVector(angularUpper));
    }


    void setLimit(int axis, irr::f32 lo, irr::f32 hi)
    {
    	m_fixedConstraint->setLimit(axis,lo,hi);
    }

    bool	isLimited(int limitIndex)
    {
    	return m_fixedConstraint->isLimited(limitIndex);
    }


	bool getUseFrameOffset() { return m_fixedConstraint->getUseFrameOffset(); }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_fixedConstraint->setUseFrameOffset(frameOffsetOnOff); }

	void enableSpring(int index, bool onOff) { m_fixedConstraint->enableSpring(index,onOff); }
	void setStiffness(int index, irr::f32 stiffness){ m_fixedConstraint->setStiffness(index,stiffness); }
	void setDamping(int index, irr::f32 damping){ m_fixedConstraint->setDamping(index,damping); }
	void setEquilibriumPoint(){ m_fixedConstraint->setEquilibriumPoint(); } 
	void setEquilibriumPoint(int index){ m_fixedConstraint->setEquilibriumPoint(index); }
	void setEquilibriumPoint(int index, irr::f32 val){ m_fixedConstraint->setEquilibriumPoint(index,val); } 

	void setAxis( const irr::core::vector3df& axis1, const irr::core::vector3df& axis2)
	{
		m_fixedConstraint->setAxis(bullet::irrVectorToBulletVector(axis1),bullet::irrVectorToBulletVector(axis2));
	}

	void drop(){delete this;}
private:
	btGeneric6DofSpringConstraint * m_fixedConstraint;
};

#endif