#include <private/global_options.h>

#include <QString>

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
{
    mLayout.addWidget(&mSolve);
    mLayout.addWidget(&mInitialTime);
    mLayout.addWidget(&mFinalTime);
    mLayout.addWidget(&mDeltaTime);
    setLayout(&mLayout);

    connect(
        &mSolve, &QPushButton::pressed,
        this, &GlobalOptions::requestSolver
    );
}

void
GlobalOptions::requestSolver(

){
    float t0 = mInitialTime.getValue();
    float t1 = mFinalTime.getValue();
    float dt = mDeltaTime.getValue();
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