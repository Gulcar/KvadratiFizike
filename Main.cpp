#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <array>
#include <iostream>
#include <vector>
#include <math.h>
#include <cassert>
#include <thread>
#include <chrono>

struct Kvadrat
{
    sf::Vector2f position;
    sf::Vector2f velocity;

    float rotation;
    float angularVelocity;

    float width;

    sf::Color color;

    float elasticity = 0.2f;
    float mass = 1.0f;
    float inertia = 10000.0f;
};

sf::VertexArray debugLines(sf::Lines);
std::vector<sf::Vector2f> debugCircles;
sf::Vector2f gravity = sf::Vector2f(0.0f, 3000.0f);
float damping = 0.6f;
float angularDamping = 1.0f;

float Dot(sf::Vector2f a, sf::Vector2f b)
{
    return a.x * b.x + a.y * b.y;
}

float Cross(sf::Vector2f a, sf::Vector2f b)
{
    return a.x * b.y - a.y * b.x;
}

float Length(sf::Vector2f a)
{
    return std::sqrt(a.x * a.x + a.y * a.y);
}

sf::Vector2f Normalize(sf::Vector2f a)
{
    float len = Length(a);
    return a / len;
}

void Integrate(Kvadrat& kvadrat, float dt)
{
    sf::Vector2f acceleration = gravity - damping * kvadrat.velocity / kvadrat.mass;

    kvadrat.velocity += acceleration * dt;
    kvadrat.position += kvadrat.velocity * dt;

    float angularAcceleration = - angularDamping * kvadrat.angularVelocity / kvadrat.inertia;
    kvadrat.angularVelocity += angularAcceleration * dt;
    kvadrat.rotation += kvadrat.angularVelocity * dt;
}

void IntegrateWithSpring(Kvadrat& kvadrat, float dt, float k, sf::Vector2f x, sf::Vector2f r)
{
    sf::Vector2f acceleration = gravity - damping * kvadrat.velocity / kvadrat.mass;
    sf::Vector2f springForce = -k * x;
    acceleration += springForce / kvadrat.mass;

    kvadrat.velocity += acceleration * dt;
    kvadrat.position += kvadrat.velocity * dt;

    float torque = Cross(r, springForce);
    float angularAcceleration = - angularDamping * kvadrat.angularVelocity / kvadrat.inertia;
    angularAcceleration += torque / kvadrat.inertia;
    kvadrat.angularVelocity += angularAcceleration * dt;
    kvadrat.rotation += kvadrat.angularVelocity * dt;
}


void Draw(Kvadrat& kvadrat, sf::RenderWindow& window)
{
    sf::RectangleShape rect;
    rect.setPosition(kvadrat.position);
    rect.setSize({ kvadrat.width, kvadrat.width });
    rect.setFillColor(kvadrat.color);
    rect.setOrigin(rect.getSize() / 2.0f);
    rect.setRotation(kvadrat.rotation * 180.0f / 3.14159f);

    window.draw(rect);
}

std::array<sf::Vector2f, 4> GetCorners(const Kvadrat& k)
{
    float angle = k.rotation;
    sf::Vector2f up = sf::Vector2f(cosf(angle), sinf(angle));

    angle += 3.14159f / 2.0f;
    sf::Vector2f right = sf::Vector2f(cosf(angle), sinf(angle));

    std::array<sf::Vector2f, 4> corners = {
        k.position + right * k.width / 2.0f + up * k.width / 2.0f,
        k.position - right * k.width / 2.0f + up * k.width / 2.0f,
        k.position - right * k.width / 2.0f - up * k.width / 2.0f,
        k.position + right * k.width / 2.0f - up * k.width / 2.0f,
    };

    return corners;
}

sf::Vector2f RotatePoint(sf::Vector2f p, float angle)
{
    return {
        p.x * cosf(angle) - p.y * sinf(angle),
        p.x * sinf(angle) + p.y * cosf(angle)
    };
}

bool IsPointInKvadrat(sf::Vector2f point, const Kvadrat& kvadrat)
{
    sf::Vector2f localPoint = point - kvadrat.position;
    localPoint = RotatePoint(localPoint, -kvadrat.rotation);

    return (localPoint.x < kvadrat.width / 2.0f && localPoint.x > -kvadrat.width / 2.0f &&
            localPoint.y < kvadrat.width / 2.0f && localPoint.y > -kvadrat.width / 2.0f);
}

std::pair<float, float> ProjectOnAxis(Kvadrat& k, sf::Vector2f axis)
{
    float min = HUGE_VALF;
    float max = -HUGE_VALF;

    auto corners = GetCorners(k);

    for (const auto& corner : corners)
    {
        //debugCircles.push_back(corner);

        float p = Dot(corner, axis);

        min = std::min(p, min);
        max = std::max(p, max);
    }

    return { min, max };
}

bool AreColliding(Kvadrat& k1, Kvadrat& k2, sf::Vector2f* outNormal, float* outPenetration)
{
    float a1 = k1.rotation;
    float a2 = k2.rotation;

    std::array<sf::Vector2f, 4> axises = {
        sf::Vector2f(cosf(a1), sinf(a1)),
        sf::Vector2f(cosf(a1+1.570795f), sinf(a1+1.570795f)),
        sf::Vector2f(cosf(a2), sinf(a2)),
        sf::Vector2f(cosf(a2+1.570795f), sinf(a2+1.570795f)),
    };

    for (int i = 0; i < 2; i++)
    {
        debugLines.append(sf::Vertex(k1.position, sf::Color::Red));
        debugLines.append(sf::Vertex(k1.position + axises[i] * 300.0f, sf::Color::Red));
    }
    for (int i = 2; i < 4; i++)
    {
        debugLines.append(sf::Vertex(k2.position, sf::Color::Red));
        debugLines.append(sf::Vertex(k2.position + axises[i] * 300.0f, sf::Color::Red));
    }

    float penetration = HUGE_VALF;
    sf::Vector2f normal;

    for (const auto& axis : axises)
    {
        auto [min1, max1] = ProjectOnAxis(k1, axis);
        auto [min2, max2] = ProjectOnAxis(k2, axis);

        // min1 max1 min2 max2
        // min2 max2 min1 max2

        if (max1 < min2 || max2 < min1)
            return false;

        float pen = std::min(max2 - min1, max1 - min2);
        if (pen < penetration)
        {
            penetration = pen;
            normal = axis;
        }
    }

    if (Dot(k1.position - k2.position, normal) < 0.f)
    {
        normal = -normal;
    }

    *outNormal = normal;
    *outPenetration = penetration;

    return true;
}

void HandleEdgeCollision(Kvadrat& k)
{
    auto corners = GetCorners(k);

    struct Contact
    {
        sf::Vector2f normal;
        sf::Vector2f point;
        bool twoContacts = false;
    };
    static std::vector<Contact> contacts;
    contacts.clear();

    for (const auto& corner : corners)
    {
        if (corner.x < 0.0f)
        {
            k.position.x -= corner.x;
            Contact c;
            c.point = sf::Vector2f(0.0f, corner.y);
            c.normal = { 1.0f, 0.0f };
            contacts.push_back(c);
        }
        else if (corner.x > 1600.0f)
        {
            k.position.x -= corner.x - 1600.0f;
            Contact c;
            c.point = sf::Vector2f(1600.0f, corner.y);
            c.normal = { -1.0f, 0.0f };
            contacts.push_back(c);
        }
        else if (corner.y < 0.0f)
        {
            k.position.y -= corner.y;
            Contact c;
            c.point = sf::Vector2f(corner.x, 0.0f);
            c.normal = { 0.0f, 1.0f };
            contacts.push_back(c);
        }
        else if (corner.y > 900.0f)
        {
            k.position.y -= corner.y - 900.0f;
            Contact c;
            c.point = sf::Vector2f(corner.x, 900.0f);
            c.normal = { 0.0f, -1.0f };
            contacts.push_back(c);
        }
    }

    for (int i = 0; i < (int)contacts.size() - 1; i++)
    {
        for (int j = i + 1; j < (int)contacts.size(); j++)
        {
            if (contacts[i].normal == contacts[j].normal)
            {
                contacts[i].point = (contacts[i].point + contacts[j].point) / 2.0f;
                contacts[i].twoContacts = true;
                contacts.erase(contacts.begin() + j);
                break;
            }
        }
    }

    for (auto& contact : contacts)
    {
        //k.velocity += -(1.0f + k.elasticity) * normal * Dot(k.velocity, normal);

        // https://en.wikipedia.org/wiki/Collision_response
        
        sf::Vector2f& n = contact.normal;

        sf::Vector2f r_ap = contact.point - k.position;
        sf::Vector2f v_ap = k.velocity + sf::Vector2f(-k.angularVelocity * r_ap.y, k.angularVelocity * r_ap.x);

        float j = -(1.0f + k.elasticity) * Dot(v_ap, n);
        j /= (1.0f / k.mass) + Cross(r_ap, n) * Cross(r_ap, n) / k.inertia;

        // da se ne trese na tleh
        if (contact.twoContacts && j < 150.0f)
        {
            k.velocity -= Dot(k.velocity, n) * n;
            k.angularVelocity = 0.0f;
        }
        else
        {
            k.velocity += j * n / k.mass;
            k.angularVelocity += Cross(r_ap, j * n) / k.inertia;
        }


        debugLines.append(sf::Vertex(contact.point, sf::Color::Green));
        debugLines.append(sf::Vertex(contact.point + n * 100.0f, sf::Color::Green));
    }
}

float PointLineDistance(sf::Vector2f point, sf::Vector2f lineA, sf::Vector2f lineB)
{
    sf::Vector2f dir = Normalize(lineA - lineB);
    float proj = Dot(point - lineB, dir);

    if (proj < 0.0f) proj = 0.0f;
    else if (proj > Length(lineA - lineB)) proj = Length(lineA - lineB);

    sf::Vector2f closestPoint = proj * dir + lineB;

    return Length(closestPoint - point);
}

sf::Vector2f GetContactPoint(Kvadrat& k1, Kvadrat& k2)
{
    float closest = HUGE_VALF;
    sf::Vector2f point;

    auto corners1 = GetCorners(k1);
    auto corners2 = GetCorners(k2);

    for (auto& corner : corners1)
    {
        for (int i = 0; i < 4; i++)
        {
            int j = i + 1;
            if (j > 3) j = 0;

            float dist = PointLineDistance(corner, corners2[i], corners2[j]);
            if (dist < closest)
            {
                closest = dist;
                point = corner;
            }
        }
    }
    for (auto& corner : corners2)
    {
        for (int i = 0; i < 4; i++)
        {
            int j = i + 1;
            if (j > 3) j = 0;

            float dist = PointLineDistance(corner, corners1[i], corners1[j]);
            if (dist < closest)
            {
                closest = dist;
                point = corner;
            }
        }
    }

    debugCircles.push_back(point);

    return point;
}

void ResolveCollision(Kvadrat& k1, Kvadrat& k2, sf::Vector2f normal, float penetration)
{
    k1.position += normal * penetration / 2.0f;
    k2.position -= normal * penetration / 2.0f;

    sf::Vector2f contactPoint = GetContactPoint(k1, k2);

    debugLines.append(sf::Vertex(contactPoint, sf::Color::Yellow));
    debugLines.append(sf::Vertex(contactPoint + normal * 100.0f, sf::Color::Yellow));
    debugCircles.push_back(contactPoint);

    float e = std::min(k1.elasticity, k2.elasticity);
    sf::Vector2f& n = normal;

    sf::Vector2f r_ap = contactPoint - k1.position;
    sf::Vector2f r_bp = contactPoint - k2.position;

    sf::Vector2f v_ap = k1.velocity + sf::Vector2f(-k1.angularVelocity * r_ap.y, k1.angularVelocity * r_ap.x);
    sf::Vector2f v_bp = k2.velocity + sf::Vector2f(-k2.angularVelocity * r_bp.y, k2.angularVelocity * r_bp.x);

    sf::Vector2f v_ab = v_ap - v_bp;

    float j = -(1.0f + e) * Dot(v_ab, n);
    j /= (1.0f / k1.mass) + (1.0f / k2.mass) + Cross(r_ap, n) * Cross(r_ap, n) / k1.inertia
        + Cross(r_bp, n) * Cross(r_bp, n) / k2.inertia;

    k1.velocity += j * n / k1.mass;
    k2.velocity -= j * n / k2.mass;

    k1.angularVelocity += Cross(r_ap, j * n) / k1.inertia;
    k2.angularVelocity -= Cross(r_bp, j * n) / k2.inertia;
}

void DodajKvadrate(std::vector<Kvadrat>& vec)
{
    Kvadrat k = {};
    k.position = { 500.0f, 500.0f };
    k.rotation = 0.0f;
    k.angularVelocity = 0.0f;
    k.width = 150.0f;
    k.color = sf::Color::Blue;

    Kvadrat k1 = {};
    k1.position = { 900.0f, 300.0f };
    k1.velocity = { -100.0f, -200.0f };
    k1.rotation = -2.0f;
    k1.angularVelocity = 0.5f;
    k1.width = 150.0f;
    k1.color = sf::Color::Blue;

    Kvadrat k2 = {};
    k2.position = { 300.0f, 700.0f };
    k2.velocity = { 100.0f, 200.0f };
    k2.rotation = 3.0f;
    k2.angularVelocity = -0.5f;
    k2.width = 150.0f;
    k2.color = sf::Color::Blue;

    Kvadrat k3 = {};
    k3.position = { 150.0f, 300.0f };
    k3.rotation = 6.0f;
    k3.angularVelocity = 0.0f;
    k3.width = 150.0f;
    k3.color = sf::Color::Blue;

    vec.reserve(vec.size() + 3);
    vec.push_back(k);
    vec.push_back(k1);
    vec.push_back(k2);
    vec.push_back(k3);
}

sf::Vector2f mouseHoldOffset;
int mouseHoldIndex = -1;
bool showDebugGfx = true;

void HandleEvent(sf::Event& e, sf::RenderWindow& window, std::vector<Kvadrat>& kvadrati)
{
    switch (e.type)
    {
        case sf::Event::Closed:
            window.close();
            break;

        case sf::Event::KeyPressed:
            if (e.key.code == sf::Keyboard::Escape)
                window.close();
            else if (e.key.code == sf::Keyboard::D)
                showDebugGfx = !showDebugGfx;
            break;

        case sf::Event::MouseButtonPressed:
            if (e.mouseButton.button == 0)
            {
                sf::Vector2f mousePos = { (float)e.mouseButton.x, (float)e.mouseButton.y };
                for (size_t i = 0; i < kvadrati.size(); i++)
                {
                    if (IsPointInKvadrat(mousePos, kvadrati[i]))
                    {
                        mouseHoldIndex = i;
                        mouseHoldOffset = RotatePoint(mousePos - kvadrati[i].position, -kvadrati[i].rotation);
                        break;
                    }
                }
            }
            break;

        case sf::Event::MouseButtonReleased:
            if (e.mouseButton.button == 0)
            {
                mouseHoldIndex = -1;
            }
            break;

        default:
            break;
    }
}

void IntegrateAll(std::vector<Kvadrat>& kvadrati, float dt, sf::RenderWindow& window)
{
    for (int i = 0; i < (int)kvadrati.size(); i++)
    {
        if (i == mouseHoldIndex)
        {
            sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            sf::Vector2f offset = RotatePoint(mouseHoldOffset, kvadrati[i].rotation);
            sf::Vector2f pos = kvadrati[i].position + offset;
            debugCircles.push_back(pos);
            debugCircles.push_back(mousePos);
            debugLines.append(sf::Vertex(mousePos, sf::Color::Magenta));
            debugLines.append(sf::Vertex(pos, sf::Color::Magenta));

            IntegrateWithSpring(kvadrati[i], dt, 15.0f, pos - mousePos, offset);
        }
        else
        {
            Integrate(kvadrati[i], dt);
        }
    }
}

void UpdateCollisions(std::vector<Kvadrat>& kvadrati)
{
    for (auto& k : kvadrati)
        k.color = sf::Color::Blue;

    for (size_t i = 0; i < kvadrati.size() - 1; i++)
    {
        for (size_t j = i + 1; j < kvadrati.size(); j++)
        {
            sf::Vector2f normal;
            float penetration;

            if (AreColliding(kvadrati[i], kvadrati[j], &normal, &penetration))
            {
                kvadrati[i].color = sf::Color::Red;
                kvadrati[j].color = sf::Color::Red;
                ResolveCollision(kvadrati[i], kvadrati[j], normal, penetration);
            }
        }
    }

    for (auto& k : kvadrati)
        HandleEdgeCollision(k);
}

void DrawDebugGfx(sf::RenderWindow& window)
{
    for (const auto& c : debugCircles)
    {
        sf::CircleShape circle;
        circle.setPosition(c);
        circle.setRadius(8.0f);
        circle.setOutlineColor(sf::Color::Red);
        circle.setOrigin({ circle.getRadius(), circle.getRadius() });
        window.draw(circle);
    }

    if (showDebugGfx)
        window.draw(debugLines);
}

int main()
{
    std::cout << "pozdravljen svet\n";
    sf::RenderWindow window(sf::VideoMode(1600, 900), "kvadrati-fizike");
    window.setVerticalSyncEnabled(true);

    std::vector<Kvadrat> kvadrati;
    DodajKvadrate(kvadrati);

    sf::Clock clock;

    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 0.03f);
        debugLines.clear();
        debugCircles.clear();

        sf::Event e;
        while (window.pollEvent(e))
        {
            HandleEvent(e, window, kvadrati);
        }


        int steps = 5;
        for (int i = 0; i < steps; i++)
        {
            IntegrateAll(kvadrati, dt / steps, window);
            UpdateCollisions(kvadrati);
        }

        window.clear(sf::Color(17, 17, 17));

        for (auto& k : kvadrati)
            Draw(k, window);

        if (showDebugGfx)
            DrawDebugGfx(window);

        window.display();

        //std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}
