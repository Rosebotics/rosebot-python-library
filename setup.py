from setuptools import setup

setup(
    name='rosebot',
    version='1.0',
    packages=['rosebot'],
    install_requires=['pymata_aio>=2.4'],
    url='http://www.rosebotics.org',
    download_url='https://github.com/Rosebotics/rosebot-python-library',
    license='GNU General Public License v3 (GPLv3)',
    author='David Fisher',
    author_email='fisherds@gmail.com',
    description='A pymata-aio wrapper library used at Rose-Hulman for the RoseBot',
    keywords=['Firmata', 'Arduino'],
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3 :: Only',
        'Topic :: Utilities',
        'Topic :: Education',
        'Topic :: Home Automation',
    ],
)