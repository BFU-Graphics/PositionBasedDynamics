#ifndef POSITIONBASEDDYNAMICS_CONTACTS_H
#define POSITIONBASEDDYNAMICS_CONTACTS_H


namespace HINASIM
{
    class ContactConstraint
    {
    public:
//        virtual bool solve();

    public:
        double stiffness_ = 1.0;
    };

    class RigidBodyContactConstraint : public ContactConstraint
    {

    };
}

#endif //POSITIONBASEDDYNAMICS_CONTACTS_H
