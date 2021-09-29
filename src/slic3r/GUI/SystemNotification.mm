#import <Foundation/Foundation.h>
#import "Plater.hpp"

void Plater::post_mac_notifiacation(const std::string &title, const std::string &message)
{
    NSString * ttl_nss = [NSString stringWithCString:title.c_str() encoding:[NSString defaultCStringEncoding]];
    NSString * msg_nss = [NSString stringWithCString:message.c_str() encoding:[NSString defaultCStringEncoding]];
    NSUserNotification* notification = [[NSUserNotification alloc] init];
    notification.title = ttl_nss;
    notification.informativeText = msg_nss;
    notification.soundName = NSUserNotificationDefaultSoundName;   //Will play a default sound
    [[NSUserNotificationCenter defaultUserNotificationCenter] deliverNotification: notification];
    [notification autorelease];
}