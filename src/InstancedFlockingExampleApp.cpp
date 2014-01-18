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
#include "cinder/gl/Light.h"
#include "cinder/Camera.h"
#include "cinder/gl/Shader.h"

#include "Resources.h"

#define NUMBOIDS 20000

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
    //mTransform.scale(.5);
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
   // Vec3f separateForce = separate(particles);
    Vec3f seekForce = seek(mCurrentTarget);
   // addForce(separateForce);
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
    float theta = acos(mLocation.dot(mCurrentTarget) / (mLocation.length() * mCurrentTarget.length()));
    mTransform.rotate(axis,toRadians(theta));
   // mTransform.setScale( Vec3f::one() * .75f );//* Vec3f( 0.1f, 150.0f, 0.1f ) );
    return mTransform;
}


class InstancedFlockingExampleApp : public AppNative {
public:
	void setup();
	void mouseDown( MouseEvent event );
	void update();
	void draw();
    void prepareSettings(Settings* settings);
    
    vector<Particle*> particles;
    gl::GlslProgRef renderShader;
    gl::Light light;
    gl::VaoRef mVao;
    gl::VboRef mVbo;
    gl::VboRef mElemVbo;
    gl::VboRef mTransformBuffer;
    CameraPersp mCam;
    Perlin perlin;
    float mNoiseScale;
};

void InstancedFlockingExampleApp::prepareSettings( Settings * settings )
{
    settings->setWindowSize(Vec2i(1280,800));
}

void InstancedFlockingExampleApp::setup()
{
    particles.resize(0);
    for(int i=0;i<NUMBOIDS;i++){
        Particle* p = new Particle(randVec3f()*100.f, Vec3f::zero(), i);
        particles.push_back(p);
    }
    
    mCam.setPerspective(60, getWindowAspectRatio(), 1., 1000000.);
    mCam.lookAt(Vec3f(0.,0.,100.),Vec3f::zero(), Vec3f::yAxis());
    
    light = gl::Light(gl::Light::LightType::POINT, 0);
    light.setAmbient(ColorA(0.,0.,0.,1.));
    light.setDiffuse(ColorA(.9,.9,.9,1.));
    light.setSpecular(ColorA(1.,1.,1.,1.));
    light.setShine(128);
    light.setConstantAttenuation(0.);
    light.setLinearAttenuation(0.);
    
    mNoiseScale = 20.f;
    
    glPointSize(4.);
    
    const Vec3f v0 = Vec3f( -1.,-1., -1.);
    const Vec3f v1 = Vec3f( -1., 1., -1.);
    const Vec3f v2 = Vec3f( 1., -1., -1.);
    const Vec3f v3 = Vec3f( 1.,  1., -1.);
    const Vec3f v4 = Vec3f( 1., -1.,  1.);
    const Vec3f v5 = Vec3f( 1.,  1.,  1.);
    const Vec3f v6 = Vec3f( -1.,-1.,  1.);
    const Vec3f v7 = Vec3f(-1.,  1.,  1.);
    
    Vec3f tmp1 = Vec3f();
    Vec3f tmp2 = Vec3f();
    Vec3f tmp3 = Vec3f();

    tmp1 = cross((v0-v2),(v1-v2)).normalized();
    tmp2 = cross((v6-v0),(v7-v0)).normalized();
    tmp3 = cross((v0-v6),(v2-v6)).normalized();
    Vec3f n0 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v0-v2),(v1-v2)).normalized();
    tmp2 = cross((v7-v1),(v3-v1)).normalized();
    tmp3 = cross((v0-v1),(v7-v1)).normalized();
    Vec3f n1 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v0-v2),(v1-v2)).normalized();
    tmp2 = cross((v2-v4),(v3-v4)).normalized();
    tmp3 = cross((v2-v6),(v4-v6)).normalized();
    Vec3f n2 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v2-v3),(v1-v3)).normalized();
    tmp2 = cross((v2-v4),(v3-v4)).normalized();
    tmp3 = cross((v7-v3),(v5-v3)).normalized();
    Vec3f n3 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v4-v5),(v3-v5)).normalized();
    tmp2 = cross((v4-v6),(v5-v6)).normalized();
    tmp3 = cross((v2-v6),(v4-v6)).normalized();
    Vec3f n4 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v4-v5),(v3-v5)).normalized();
    tmp2 = cross((v7-v3),(v5-v3)).normalized();
    tmp3 = cross((v4-v6),(v5-v6)).normalized();
    Vec3f n5 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v6-v7),(v5-v7)).normalized();
    tmp2 = cross((v6-v0),(v7-v0)).normalized();
    tmp3 = cross((v6-v2),(v0-v2)).normalized();
    Vec3f n6 = (tmp1 + tmp2 + tmp3)/3;
    
    tmp1 = cross((v7-v3),(v5-v3)).normalized();
    tmp2 = cross((v6-v7),(v5-v7)).normalized();
    tmp3 = cross((v6-v0),(v7-v0)).normalized();
    Vec3f n7 = (tmp1 + tmp2 + tmp3)/3;
    
    static const Vertex verts[] ={
        Vertex(v0,n0),
        Vertex(v1,n1),
        Vertex(v2,n2),
        Vertex(v3,n3),
        Vertex(v4,n4),
        Vertex(v5,n5),
        Vertex(v6,n6),
        Vertex(v7,n7)
    };
    
   static const GLushort faces[] = {
        0, 1, 2,  // cross((v1-v2),(v0-v2)).normalized();
        2, 1, 3,  // cross((v1-v2),(v3-v2)).normalized();
        2, 3, 4,  // cross((v3-v2),(v4-v2)).normalized();
        4, 3, 5,  // cross((v3-v4),(v5-v4)).normalized();
        4, 5, 6,  // cross((v5-v4),(v6-v4)).normalized();
        6, 5, 7,  // cross((v5-v6),(v7-v6)).normalized();
        6, 7, 0,  // cross((v7-v6),(v0-v6)).normalized();
        0, 7, 1,  // cross((v7-v0),(v1-v0)).normalized();
        6, 0, 2,  // cross((v0-v6),(v2-v6)).normalized();
        2, 4, 6,  // cross((v4-v2),(v6-v2)).normalized();
        7, 5, 3,  // cross((v5-v7),(v3-v7)).normalized();
        7, 3, 1   // cross((v3-v7),(v1-v7)).normalized();
        
    };
    
    vector< Matrix44f > matrices;
    for( int i = 0; i < NUMBOIDS; i++ ){
        Matrix44f mat;
        mat.translate( randVec3f() * randFloat( 1, 500 ) );
        mat.rotate( randVec3f() * 50.0f );
        mat.scale( Vec3f::one() * .1f );//* Vec3f( 0.1f, 150.0f, 0.1f ) );
        matrices.push_back( mat );
    }
    
    
    mVao = gl::Vao::create();
    mVbo = gl::Vbo::create( GL_ARRAY_BUFFER, sizeof(verts), verts);
    mElemVbo = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, sizeof(faces), faces);
    
    mVao->bind();
    mVbo->bind();
    gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex,p));
    gl::enableVertexAttribArray(0);
    gl::vertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex,n));
    gl::enableVertexAttribArray(1);
    mVbo->unbind();
    
    gl::GlslProg::Format mFormat;
	mFormat.vertex( loadResource( LIGHT_VS ) );
	mFormat.fragment( loadResource( LIGHT_FS ) );
	
	try {
		renderShader = gl::GlslProg::create( mFormat );
	}
	catch( gl::GlslProgCompileExc ex ) {
		cout << ex.what() << endl;
	}
    
    GLint ulocation = 2;
    
    mTransformBuffer = gl::Vbo::create(GL_ARRAY_BUFFER, matrices.size()*sizeof(Matrix44f), matrices.data());
    mTransformBuffer->bind();
	for (unsigned int i = 0; i < 4 ; i++) {
        glEnableVertexAttribArray(ulocation + i);
        glVertexAttribPointer(ulocation + i, 4, GL_FLOAT, GL_FALSE, sizeof(Matrix44f), (const GLvoid*)(sizeof(GLfloat) * i * 4));
        glVertexAttribDivisor(ulocation + i, 1);
    }
    mTransformBuffer->unbind();
    
    mVao->unbind();

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
   // Vec3f n = Vec3f(0.,0.,0.);
    light.setPosition(n);
    
    uint8_t *data = mTransformBuffer->map( GL_WRITE_ONLY );
    uint8_t stride = sizeof(Matrix44f);
    for( auto iter = particles.begin(); iter != particles.end(); ++iter ){
        (*iter)->updateTarget(light.getPosition());
        (*iter)->applyBehaviors(particles);
        (*iter)->update();
        
        Matrix44f *ptr = reinterpret_cast<Matrix44f*>( &data[0] );
        if( ptr != NULL ){
            *ptr = (*iter)->getData();
            data += stride;
        }
        
    }
	mTransformBuffer->unmap();
	mTransformBuffer->unbind();
    
    i++;
    
    WindowRef window = getWindow();
    window->setTitle( to_string(getAverageFps()) );
}

void InstancedFlockingExampleApp::draw()
{

    
    static int i = 0;
	// clear out the window with black
	gl::clear( Color( 0., 0., 0. ) );
    gl::pushMatrices();
    gl::setMatrices(mCam);
    gl::multModelView(Matrix44f::createRotation(Vec3f(0,1,0), toRadians((float)i)));
    mVao->bind();
    mElemVbo->bind();
    renderShader->bind();
    renderShader->uniform("p", gl::getProjection());
    renderShader->uniform("mv", gl::getModelView());
    renderShader->uniform("light", light.getPosition());
    glDrawElementsInstanced(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, 0, NUMBOIDS);
    renderShader->unbind();
    mElemVbo->unbind();
    mVao->unbind();
   
    gl::popMatrices();
    i++;
}

CINDER_APP_NATIVE( InstancedFlockingExampleApp, RendererGl )
