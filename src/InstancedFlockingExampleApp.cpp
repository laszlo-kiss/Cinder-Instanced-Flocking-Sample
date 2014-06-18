#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vao.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Rand.h"
#include "cinder/Perlin.h"
#include "cinder/Matrix.h"
#include "cinder/Camera.h"
#include "cinder/gl/Shader.h"
#include "cinder/TriMesh.h"
#include "cinder/GeomIo.h"
#include "cinder/gl/Context.h"
#include "cinder/ObjLoader.h"

#include "Resources.h"

const int NUMBOIDS = 300;

using namespace ci;
using namespace ci::app;
using namespace std;

struct Vertex {
    Vertex(Vec3f _p, Vec3f _n){
        p=_p;
        n=_n;
    }
    Vec3f p;
    Vec3f n;
};

class Particle {
public:
    Particle( Vec3f _loc,  Vec3f _target, int &_id );
    void update();
    void applyBehaviors(  vector<Particle*> &particles );
    void updateTarget( const Vec3f &newTarget );
    bool hasArrived();
    void addForce( const Vec3f &force);
    Matrix44f& getData();
    
    int mId;
    Vec3f mCurrentTarget;
    Matrix44f mTransform;
    
private:
    
    Vec3f seek( const Vec3f &target );
    Vec3f separate(  vector<Particle*> &particles );
    Vec3f cohesion(  vector<Particle*> &particles );
    Vec3f mLocation;
    Vec3f mStartLoc;
    Vec3f mVelocity;
    Vec3f mAcceleration;
    float mMaxforce = 5;    // Maximum steering force
    float mMaxspeed = .5;    // Maximum speed
    float mMass;
    
    
};

Particle::Particle( Vec3f _loc, Vec3f _target, int &_id )
{
    mLocation = _loc;
    mCurrentTarget = _target;
    mId = _id;
    mMaxforce = .03f;
    mMaxspeed = 1.5f;
    mVelocity = Vec3f::zero();
    mAcceleration = Vec3f::zero();
    mMass = randFloat(1., 3.);
    mTransform = Matrix44f::identity();
    float r = randFloat(.1,1.);
    mTransform.scale( Vec3f(r,r,r) * mMass );
}

void Particle::update(){
    mVelocity+=mAcceleration;
    mVelocity.limit(mMaxspeed);
    mLocation+=mVelocity;
    mAcceleration*=0;
}

void Particle::addForce( const Vec3f &force){
    mAcceleration+=force/mMass;
}

void Particle::applyBehaviors( vector<Particle*> &particles){
    Vec3f separateForce = separate(particles);
    Vec3f seekForce = seek(mCurrentTarget);
    addForce(separateForce);
    addForce(seekForce);
}

Vec3f Particle::seek(const Vec3f& target) {
    Vec3f desired = target - mLocation;
    desired.normalize();
    desired *= mMaxspeed;
    Vec3f steer = desired - mVelocity;
    steer.limit(mMaxforce);
    return steer;
}

Vec3f Particle::separate ( vector<Particle*>& particles) {
    float desiredseparation = 5.;
    Vec3f sum;
    int count = 0;
    for (int i=0;i<particles.size();i++){
        if(mId != i){
        float d = mLocation.distance(particles[i]->mLocation);
        if ((d > 0) && (d < desiredseparation)) {
            Vec3f diff = mLocation - particles[i]->mLocation;
            diff.normalize();
            diff/=d;
            sum+=diff;
            count++;
        }
        }
    }
    
    if (count > 0) {
        sum/=count;
        sum.normalize();
        sum*=mMaxspeed;
        sum -= mVelocity;
        sum.limit(mMaxforce);
    }
    return sum;
}


void Particle::updateTarget(const Vec3f &newTarget){
    mCurrentTarget = newTarget;
}

Matrix44f& Particle::getData(){
    mTransform.setTranslate(mLocation);
    Vec3f axis = cross(mLocation,mCurrentTarget).normalized();
    float theta = acos(mLocation.dot(mCurrentTarget) / (mLocation.lengthSquared() * mCurrentTarget.lengthSquared()));
    mTransform.rotate(axis,toRadians(theta));
    return mTransform;
}


class InstancedFlockingExampleApp : public AppNative {
public:
	void setup();
	void mouseDown( MouseEvent event );
    void keyDown( KeyEvent event );
	void update();
	void draw();
    void prepareSettings(Settings* settings);
    
    vector<Particle*> particles;
    gl::GlslProgRef renderShader, leader;
    Vec3f light;
    gl::VaoRef mVao, mLeaderVao;
    gl::VboRef mPosVbo, mNormVbo;
    gl::VboRef mElemVbo;
    gl::VboRef mTransformBuffer;
    CameraPersp mCam;
    Perlin perlin;
    float mNoiseScale;
    TriMeshRef tMesh;
};

void InstancedFlockingExampleApp::prepareSettings( Settings * settings )
{
    settings->setFullScreen();
}

void InstancedFlockingExampleApp::keyDown(cinder::app::KeyEvent event){
    if(event.getChar()=='f'){
        WindowRef window = getWindow();
        window->setFullScreen();
    }
}

void InstancedFlockingExampleApp::setup()
{
    particles.resize(0);
    for(int i=0;i<NUMBOIDS;i++){
        Particle* p = new Particle(randVec3f()*100.f, Vec3f::zero(), i);
        particles.push_back(p);
    }
    
    mCam.setPerspective(60, getWindowAspectRatio(), 1., 1000000.);
    mCam.lookAt(Vec3f(0.,0.,75.),Vec3f::zero(), Vec3f::yAxis());
    
    mNoiseScale = 20.f;
    
    tMesh = TriMesh::create( geom::Cube().enable(geom::Attrib::POSITION).enable(geom::Attrib::NORMAL) );
 //   ObjLoader loader( loadAsset("penis_centered.obj"));
	//tMesh = TriMesh::create(loader);
    
    mVao = gl::Vao::create();
    mPosVbo = gl::Vbo::create( GL_ARRAY_BUFFER, tMesh->getNumVertices() * sizeof(Vec3f), tMesh->getVertices<3>());
    mNormVbo = gl::Vbo::create( GL_ARRAY_BUFFER, tMesh->getNormals().size() * sizeof(Vec3f), tMesh->getNormals().data());
    mElemVbo = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, tMesh->getNumIndices()*sizeof(uint32_t), tMesh->getIndices().data() );
    
    gl::ScopedVao vao(mVao);
    {
        gl::ScopedBuffer vbo(mPosVbo);
        gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3f), 0);
        gl::enableVertexAttribArray(0);
    }
    {
        gl::ScopedBuffer vbo(mNormVbo);
        gl::vertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3f), 0);
        gl::enableVertexAttribArray(1);
    }
    gl::GlslProg::Format mFormat;
	mFormat.vertex( loadResource( LIGHT_VS ) );
	mFormat.fragment( loadResource( LIGHT_FS ) );
	
	try {
		renderShader = gl::GlslProg::create( mFormat );
	}
	catch( gl::GlslProgCompileExc ex ) {
		cout << ex.what() << endl;
	}
    
    mFormat.vertex( loadResource( LEADER_VS ) );
	mFormat.fragment( loadResource( LEADER_FS ) );
    
    try {
		leader = gl::GlslProg::create( mFormat );
	}
	catch( gl::GlslProgCompileExc ex ) {
		cout << ex.what() << endl;
	}

    //setup instanced transform matrices
    
    vector< Matrix44f > matrices;
    for( int i = 0; i < NUMBOIDS; i++ ){
        Matrix44f mat = Matrix44f::identity();
        matrices.push_back( mat );
    }
    
    
    GLint ulocation = 2;

    
    mTransformBuffer = gl::Vbo::create(GL_ARRAY_BUFFER, matrices.size()*sizeof(Matrix44f), matrices.data());
    {
        gl::ScopedBuffer trans(mTransformBuffer);
        
       //set up 4 attribute locations to represent the 4 columns of each transform matrix, only hefty grphics cards support 16 float vertex attribs
        for (unsigned int i = 0; i < 4 ; i++) {
            gl::enableVertexAttribArray(ulocation + i);
            gl::vertexAttribPointer(ulocation + i, 4, GL_FLOAT, GL_FALSE, sizeof(Matrix44f), (const GLvoid*)(sizeof(Vec4f)*i));
            glVertexAttribDivisor(ulocation + i, 1);
        }
        
    }
    
    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void InstancedFlockingExampleApp::mouseDown( MouseEvent event )
{
    
}

void InstancedFlockingExampleApp::update()
{
    static float i = 0;
    Vec3f n = Vec3f(mNoiseScale*cos(toRadians(i)),mNoiseScale*sin(toRadians(i)),mNoiseScale*cos(toRadians(i*2)));
    light = n;
    
    Matrix44f *data = (Matrix44f*)mTransformBuffer->map( GL_WRITE_ONLY );
    for( auto iter = particles.begin(); iter != particles.end(); ++iter ){
        (*iter)->updateTarget(light);
        (*iter)->applyBehaviors(particles);
        (*iter)->update();
        
        if( data != NULL ){
            *data++ = (*iter)->getData();
            
        }
        
    }
	mTransformBuffer->unmap();
    
    i++;
    
    WindowRef window = getWindow();
    window->setTitle( to_string(getAverageFps()) );
}

void InstancedFlockingExampleApp::draw()
{

    
    static int i = 0;
	// clear out the window with black
	gl::clear( Color( 0.1, 0.1, 0.1 ) );
    gl::pushMatrices();
    gl::setMatrices(mCam);
    gl::multModelMatrix(Matrix44f::createRotation(Vec3f(0,1,0), toRadians((float)i)));
    gl::ScopedVao vao(mVao);
    gl::ScopedBuffer elements(mElemVbo);
    {
        gl::ScopedGlslProg glsl(renderShader);
        renderShader->uniform("p", gl::getProjectionMatrix());
        renderShader->uniform("mv", gl::getModelView());
        renderShader->uniform("light", light);
        glDrawElementsInstanced(GL_TRIANGLES, tMesh->getNumIndices(), GL_UNSIGNED_INT, 0, NUMBOIDS);
    }
    
    {
        Matrix44f mat;
        mat = Matrix44f::identity();
        mat.translate(light);
        mat.scale(Vec3f::one()*.25);
        gl::ScopedGlslProg glsl(leader);
        leader->uniform("p", gl::getProjectionMatrix());
        leader->uniform("mv", gl::getModelView());
        leader->uniform("light", light);
        leader->uniform("transformMat", mat);
        glDrawElements(GL_TRIANGLES,tMesh->getNumIndices(),GL_UNSIGNED_INT,0);
    }
    
    gl::popMatrices();
    i++;
}

CINDER_APP_NATIVE( InstancedFlockingExampleApp, RendererGl )
