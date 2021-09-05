#include <private/global_options.h>

#include <QString>

#include <iostream>

GlobalOptionFloat::GlobalOptionFloat(
    std::string label,
    float min,
    float max,
    float initial,
    std::string description,
    QWidget* parent
):
QWidget(parent),
mLabel(QString::fromStdString(label),this),
mEditor(QString::number(initial, 'g', 3),this),
mMin(min <= max ? min : max),
mMax(max >= min ? max : min)
{
    if(!(initial >= mMin && initial <= mMax)){
        initial = (mMax + mMin)/2;
        mEditor.setText(QString::number(initial, 'g', 3));
    }

    mLabel.setToolTip(QString::fromStdString(description));
    char msg[50];
    sprintf(msg, "Float between %+.3f and %+.3f", mMin, mMax);
    mEditor.setToolTip(QString(msg));
    mLayout.addWidget(&mLabel);
    mLayout.addWidget(&mEditor);
    setLayout(&mLayout);

    connect(
        &mEditor, &QLineEdit::returnPressed,
        this, &GlobalOptionFloat::valueChanged
    );
}

float
GlobalOptionFloat::getValue(

) const {
    return mEditor.text().toFloat();
}

GlobalOptionInt::GlobalOptionInt(
    std::string label,
    int min,
    int max,
    int initial,
    std::string description,
    QWidget* parent
):
QWidget(parent),
mLabel(QString::fromStdString(label),this),
mEditor(QString::number(initial, 'g', 3),this),
mMin(min <= max ? min : max),
mMax(max >= min ? max : min)
{
    if(!(initial >= mMin && initial <= mMax)){
        initial = (mMax + mMin)/2;
        mEditor.setText(QString::number(initial, 'g', 3));
    }

    mLabel.setToolTip(QString::fromStdString(description));
    char msg[50];
    sprintf(msg, "Float between %+d and %+d", mMin, mMax);
    mEditor.setToolTip(QString(msg));
    mLayout.addWidget(&mLabel);
    mLayout.addWidget(&mEditor);
    setLayout(&mLayout);

    connect(
        &mEditor, &QLineEdit::returnPressed,
        this, &GlobalOptionInt::valueChanged
    );
}

int
GlobalOptionInt::getValue(

) const {
    return mEditor.text().toInt();
}

GlobalOptions::GlobalOptions(
    YAML::Node const & node,
    QWidget * parent
):
QWidget(parent)
,
mNode(node)
,
mSolve(QString("Solve"))
,
mInitialTime(
    std::string("Initial time"),
    0, // min
    100, // max
    getWDefault<float>(mNode["initial_time"]), // initial
    std::string("Initial time instant in seconds")
)
,
mFinalTime(
    std::string("End time"),
    0, // min
    100, // max
    getWDefault<float>(mNode["final_time"]), // initial
    std::string("Final time instant in seconds")
)
,
mDeltaTime(
    std::string("Time step"),
    0.0001, // min
    1, // max
    getWDefault<float>(mNode["delta_time"]), // initial
    std::string("Time steps in seconds, when computing solution")
)
,
mGravity1(
    std::string("Gravity x"),
    -10000, // min
    +10000, // max
    0, // initial
    std::string("Gravity x component in kg mm / s / s")
)
,
mGravity2(
    std::string("Gravity y"),
    -10000, // min
    +10000, // max
    -9.8*1000, // initial
    std::string("Gravity y component in kg mm / s / s")
)
,
mSetLinearVelocityPlotFactor(
    std::string("x Linear vel"),
    0, // min
    1000, // max
    1, // initial
    std::string("Linear velocity factor")
)
,
mSetAngularVelocityPlotFactor(
    std::string("x Angular vel"),
    0, // min
    1000, // max
    1, // initial
    std::string("Angular velocity factor")
)
,
mSetLinearForcePlotFactor(
    std::string("x Linear force"),
    0, // min
    1000, // max
    1, // initial
    std::string("Linear force factor")
)
,
mSetAngularForcePlotFactor(
    std::string("x Angular force"),
    0, // min
    1000, // max
    1, // initial
    std::string("Angular force factor")
)
{
    mLayout.addWidget(&mSolve);
    mLayout.addWidget(&mInitialTime);
    mLayout.addWidget(&mFinalTime);
    mLayout.addWidget(&mDeltaTime);
    mLayout.addWidget(&mGravity1);
    mLayout.addWidget(&mGravity2);
    mLayout.addWidget(&mSetLinearVelocityPlotFactor);
    mLayout.addWidget(&mSetAngularVelocityPlotFactor);
    mLayout.addWidget(&mSetLinearForcePlotFactor);
    mLayout.addWidget(&mSetAngularForcePlotFactor);
    setLayout(&mLayout);

    connect(
        &mSolve, &QPushButton::pressed,
        this, &GlobalOptions::requestSolver
    );

    connect(
        &mGravity1, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updateGravity
    );
    connect(
        &mGravity2, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updateGravity
    );
    connect(
        &mSetLinearVelocityPlotFactor, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updatePlotFactors
    );
    connect(
        &mSetAngularVelocityPlotFactor, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updatePlotFactors
    );
    connect(
        &mSetLinearForcePlotFactor, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updatePlotFactors
    );
    connect(
        &mSetAngularForcePlotFactor, &GlobalOptionFloat::valueChanged,
        this, &GlobalOptions::updatePlotFactors
    );

}

YAML::Node
GlobalOptions::getNode(){

    mNode["initial_time"]["value"] = mInitialTime.getValue();
    mNode["final_time"]["value"] = mFinalTime.getValue();
    mNode["delta_time"]["value"] = mDeltaTime.getValue();

    return mNode;
}

void
GlobalOptions::requestSolver(

){
    float t0 = mInitialTime.getValue();
    float t1 = mFinalTime.getValue();
    float dt = mDeltaTime.getValue();

    mNode["initial_time"]["value"] = t0;
    mNode["final_time"]["value"] = t1;
    mNode["delta_time"]["value"] = dt;

    if(t1 > t0){
        int unsigned steps = (t1 - t0)/dt;
        // if steps = 0, then we just look at initial condition
        // which may be of interest
        // if(steps > 0){
            emit startSolver(
                std::make_tuple(t0, dt, steps)
            );
        // }
    }
}

void
GlobalOptions::updateGravity(

){
    emit changeGravity(
        std::array<double, 2>(
            {{
                mGravity1.getValue(),
                mGravity2.getValue()
            }}
        )
    );
}

void
GlobalOptions::updatePlotFactors(

){
    emit plotFactorsChanged(
        std::make_tuple(
            std::array<double, 2>({{mSetLinearVelocityPlotFactor.getValue(),mSetAngularVelocityPlotFactor.getValue()}}),
            std::array<double, 2>({{mSetLinearForcePlotFactor.getValue(),mSetAngularForcePlotFactor.getValue()}})
        )
    );
}