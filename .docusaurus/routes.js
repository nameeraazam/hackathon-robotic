import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '4fe'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/humanoid-robotics',
    component: ComponentCreator('/blog/humanoid-robotics', '149'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/tags/ai',
    component: ComponentCreator('/blog/tags/tags/ai', 'f4f'),
    exact: true
  },
  {
    path: '/blog/tags/tags/humanoid',
    component: ComponentCreator('/blog/tags/tags/humanoid', '009'),
    exact: true
  },
  {
    path: '/blog/tags/tags/robotics',
    component: ComponentCreator('/blog/tags/tags/robotics', '485'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '059'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '713'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '501'),
            routes: [
              {
                path: '/docs/active_perception',
                component: ComponentCreator('/docs/active_perception', '1c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/aws_robomaker',
                component: ComponentCreator('/docs/aws_robomaker', '512'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai-and-humanoids',
                component: ComponentCreator('/docs/physical-ai-and-humanoids', '152'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vision_language_models',
                component: ComponentCreator('/docs/vision_language_models', '5a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-01-02-physical-ai/assessment',
                component: ComponentCreator('/docs/week-01-02-physical-ai/assessment', 'c13'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-03-05-ros2/urdf-robot-description',
                component: ComponentCreator('/docs/week-03-05-ros2/urdf-robot-description', '60a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-08-10-isaac/sim-to-real',
                component: ComponentCreator('/docs/week-08-10-isaac/sim-to-real', '6fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-11-12-humanoid/project',
                component: ComponentCreator('/docs/week-11-12-humanoid/project', '2f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-13-conversational/multimodal-interfaces',
                component: ComponentCreator('/docs/week-13-conversational/multimodal-interfaces', '498'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/week-13-conversational/whisper-speech-to-text',
                component: ComponentCreator('/docs/week-13-conversational/whisper-speech-to-text', '32e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/why_robotics',
                component: ComponentCreator('/docs/why_robotics', '76a'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
